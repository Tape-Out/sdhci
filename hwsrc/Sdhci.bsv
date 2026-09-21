package Sdhci;

// SD 主机控制器，寄存器布局照 Linux sdhci 驱动的定义（见 regmap.yaml）。命令通路与单块 PIO 数据通路：
// 命令寄存器那个字一被写，隔一拍（存下的字段这时才更新）照命令号与参数打成 48 位帧，在 SD 时钟下降沿逐位发，
// 上升沿采响应；48 位响应查 CRC7、命令号、结束位，136 位响应只查结束位；R1b 等卡放开 DAT0 才报完成。
// 带数据的命令收完响应接着走数据：写块等软件经 pio 填满 512 字节，再在 DAT0 上发、收卡的 CRC 状态令牌、等忙；
// 读块在 DAT0 上收 512 字节与 CRC16，对了才让软件经 pio 读。
// 状态只在 step 一条规则里写；驱动 volatile 字段与置中断的 show 另起一条，要排在总线访问之前

import RegIf::*;
import SdhciRegs::*;
import SdFrame::*;

typedef struct {
  Bit#(0) none;
} SdhciCfg;

interface SdPins;
  (* always_ready, result = "clk" *)    method Bit#(1) clk;
  (* always_ready, result = "cmd_o" *)  method Bit#(1) cmdO;
  (* always_ready, result = "cmd_oe" *) method Bit#(1) cmdOe;
  (* always_ready, result = "dat_o" *)  method Bit#(4) datO;
  (* always_ready, result = "dat_oe" *) method Bit#(1) datOe;
  (* always_ready, always_enabled, prefix = "" *)
  method Action lines((* port = "cmd_i" *) Bit#(1) cmd, (* port = "dat_i" *) Bit#(4) dat);
endinterface

interface SdhciIfc#(numeric type aw, numeric type dw);
  interface RegIf#(aw, dw) regs;
  interface SdPins         sd;
  (* always_ready *) method Bool irq;
endinterface

typedef enum { EIdle, ESend, EWait, ERecv, EBusy, ETail } Eng deriving (Bits, Eq);
typedef enum { DNone, DFill, DGap, DSend, DTok, DBusy, DWait, DRecv, DDrain } Dat deriving (Bits, Eq);

module mkSdhci#(SdhciCfg cfg)(SdhciIfc#(aw, dw))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 8, aw), Add#(_w0, 1, dw), Add#(_w1, 12, dw),
              Add#(_w2, 16, dw), Add#(_w3, 2, dw), Add#(_w4, 3, dw), Add#(_w5, 32, dw),
              Add#(_w6, 4, dw), Add#(_w7, 6, dw), Add#(_w8, 8, dw));

  SdhciRegsIfc#(aw, dw) r <- mkSdhciRegs;

  Reg#(Eng)       st     <- mkReg(EIdle);
  Reg#(Resp)      rt     <- mkReg(tagged RespNone);
  Reg#(Bit#(48))  txSh   <- mkReg(0);
  Reg#(UInt#(8))  txLeft <- mkReg(0);
  Reg#(Bit#(136)) rxSh   <- mkReg(0);
  Reg#(UInt#(8))  rxCnt  <- mkReg(0);
  Reg#(UInt#(16)) tmo    <- mkReg(0);
  Reg#(Bit#(120)) resp   <- mkReg(0);
  Reg#(Bit#(1))   sdclk  <- mkReg(0);
  Reg#(Bit#(1))   ckPrev <- mkReg(0);
  Reg#(Bit#(8))   divCnt <- mkReg(0);
  Reg#(Bit#(1))   cmdOr  <- mkReg(1);
  Reg#(Bit#(1))   cmdOeR <- mkReg(0);
  // step 这一拍出的事件，下一拍由 show 置进中断状态：置位与错误汇总位要在同一次读里一起出现，而汇总位只能在 show 里算
  Reg#(Bit#(11))  evs    <- mkReg(0);

  // 512 字节是一条 4096 位的移位寄存器：pio 写从低位压进四个字节、读从高位弹出四个字节，
  // DAT0 上从高位逐位发、从低位逐位收，用不着按运行时下标去改字节向量
  Reg#(Dat)        ds     <- mkReg(DNone);
  Reg#(Bit#(4096)) blk    <- mkReg(0);
  Reg#(UInt#(8))   words  <- mkReg(0);
  Reg#(UInt#(13))  dn     <- mkReg(0);
  Reg#(Bit#(16))   c16    <- mkReg(0);
  Reg#(Bit#(16))   crcRx  <- mkReg(0);
  Reg#(UInt#(16))  dtmo   <- mkReg(0);
  Reg#(Bit#(4))    tok    <- mkReg(0);
  Reg#(Bit#(1))    datOr  <- mkReg(1);
  Reg#(Bit#(1))    datOeR <- mkReg(0);

  // 写脉冲在总线方法之后才有，而 step 要在总线方法之前读寄存器：隔一拍，由 CReg 递过去
  Reg#(Bool) go[2]   <- mkCReg(2, False);
  Reg#(Bool) rst[2]  <- mkCReg(2, False);
  Reg#(Bool) rstD[2] <- mkCReg(2, False);
  // pio 的读写脉冲同样隔一拍才到 step；读出值因此要把「上一拍读过、还没弹」算进去，连着读才不重复
  Reg#(Bool)     rdP[2] <- mkCReg(2, False);
  Reg#(Bool)     wrP[2] <- mkCReg(2, False);
  Reg#(Bit#(32)) wrV[2] <- mkCReg(2, 0);

  Wire#(Bit#(1)) cmdIn <- mkBypassWire;
  Wire#(Bit#(4)) datIn <- mkBypassWire;

  Bit#(1) anyErr = r.ints_cto | r.ints_ccrc | r.ints_cend | r.ints_cidx | r.ints_dto | r.ints_dcrc | r.ints_dend;
  Bit#(32) isr = {9'b0, r.ints_dend, r.ints_dcrc, r.ints_dto, r.ints_cidx, r.ints_cend, r.ints_ccrc, r.ints_cto,
                  anyErr, 9'b0, r.ints_rdready, r.ints_wrready, 2'b0, r.ints_xfer, r.ints_cmddone};

  function Bit#(1) b(Bool x) = x ? 1 : 0;
  // pio 的字是小端：字的低字节是缓冲里靠前的那个字节
  function Bit#(32) swap(Bit#(32) x) = {x[7:0], x[15:8], x[23:16], x[31:24]};

  rule mark;
    if (r.cmd_index_wr) go[1] <= True;
    if ((r.clk_rstall_wr && r.clk_rstall_wr_val == 1) || (r.clk_rstcmd_wr && r.clk_rstcmd_wr_val == 1))
      rst[1] <= True;
    if ((r.clk_rstall_wr && r.clk_rstall_wr_val == 1) || (r.clk_rstdat_wr && r.clk_rstdat_wr_val == 1))
      rstD[1] <= True;
    if (r.pio_rd) rdP[1] <= True;
    if (r.pio_wr) begin
      wrP[1] <= True;
      wrV[1] <= r.pio_wr_val;
    end
  endrule

  rule show;
    r.resp0_in(resp[31:0]);
    r.resp1_in(resp[63:32]);
    r.resp2_in(resp[95:64]);
    r.resp3_in(zeroExtend(resp[119:96]));
    r.pio_in(swap(rdP[0] ? blk[4063:4032] : blk[4095:4064]));
    // 写完命令寄存器的下一拍 st 还没动，go 已经是真：两者之一就算在发
    r.state_cmdinh_in((st != EIdle || go[0]) ? 1 : 0);
    r.state_datinh_in(ds != DNone ? 1 : 0);
    r.state_dowrite_in((ds == DFill || ds == DGap || ds == DSend || ds == DTok || ds == DBusy) ? 1 : 0);
    r.state_doread_in((ds == DWait || ds == DRecv || ds == DDrain) ? 1 : 0);
    r.state_wrready_in(ds == DFill ? 1 : 0);
    r.state_rdready_in(ds == DDrain ? 1 : 0);
    r.state_present_in(1);
    r.state_stable_in(1);
    r.state_dat0_in(datIn[0]);
    r.state_cmdlvl_in(cmdIn);
    r.clk_stable_in(r.clk_inten);
    // 中断状态使能作用在置位端（与 gpio 同一个道理，见 regmap.md 第二节）
    Bit#(32) en = r.inten;
    Bit#(11) ev = evs;
    if (ev[0] == 1 && en[0] == 1)   r.ints_cmddone_set(1);
    if (ev[1] == 1 && en[16] == 1)  r.ints_cto_set(1);
    if (ev[2] == 1 && en[17] == 1)  r.ints_ccrc_set(1);
    if (ev[3] == 1 && en[18] == 1)  r.ints_cend_set(1);
    if (ev[4] == 1 && en[19] == 1)  r.ints_cidx_set(1);
    if (ev[5] == 1 && en[1] == 1)   r.ints_xfer_set(1);
    if (ev[6] == 1 && en[4] == 1)   r.ints_wrready_set(1);
    if (ev[7] == 1 && en[5] == 1)   r.ints_rdready_set(1);
    if (ev[8] == 1 && en[20] == 1)  r.ints_dto_set(1);
    if (ev[9] == 1 && en[21] == 1)  r.ints_dcrc_set(1);
    if (ev[10] == 1 && en[22] == 1) r.ints_dend_set(1);
    Bool errNow = (ev[1] == 1 && en[16] == 1) || (ev[2] == 1 && en[17] == 1)
               || (ev[3] == 1 && en[18] == 1) || (ev[4] == 1 && en[19] == 1)
               || (ev[8] == 1 && en[20] == 1) || (ev[9] == 1 && en[21] == 1) || (ev[10] == 1 && en[22] == 1);
    r.ints_err_in((anyErr == 1 || errNow) ? 1 : 0);
    r.caps_in(32'h0100_0000);
    r.ver_in(32'h0002_0000);
  endrule

  rule step;
    Eng       s   = st;
    Resp      t   = rt;
    Bit#(48)  ts  = txSh;
    UInt#(8)  tl  = txLeft;
    Bit#(136) rs  = rxSh;
    UInt#(8)  rc  = rxCnt;
    UInt#(16) to  = tmo;
    Bit#(120) rp  = resp;
    Bit#(1)   co  = cmdOr;
    Bit#(1)   coe = cmdOeR;
    Bit#(1)   ck  = sdclk;
    Bit#(8)   dc  = divCnt;
    Bool done = False;
    Bool eTo  = False;
    Bool eCrc = False;
    Bool eIdx = False;
    Bool eEnd = False;

    Dat        dsl  = ds;
    Bit#(4096) bk   = blk;
    UInt#(8)   wn   = words;
    UInt#(13)  nn   = dn;
    Bit#(16)   cr16 = c16;
    Bit#(16)   crx  = crcRx;
    UInt#(16)  dt   = dtmo;
    Bit#(4)    tk   = tok;
    Bit#(1)    dout = datOr;
    Bit#(1)    doe  = datOeR;
    Bool xfer  = False;
    Bool evWr  = False;
    Bool evRd  = False;
    Bool eDto  = False;
    Bool eDcrc = False;
    Bool eDend = False;

    // 卡时钟使能时每 div+1 拍翻一次（分频的含义自定，SDHCI 的分频模式本版不做）
    if (r.clk_carden == 1) begin
      if (dc >= r.clk_div) begin
        dc = 0;
        ck = ~ck;
      end else
        dc = dc + 1;
    end
    Bool rise = sdclk == 1 && ckPrev == 0;
    Bool fall = sdclk == 0 && ckPrev == 1;

    if (rst[0]) begin
      s   = EIdle;
      coe = 0;
      co  = 1;
      tl  = 0;
      rc  = 0;
      to  = 0;
    end else if (go[0] && s == EIdle) begin
      t  = respOf(r.cmd_resp, r.cmd_crc == 1, r.cmd_idx == 1);
      ts = cmdFrame(r.cmd_index, r.arg);
      tl = 48;
      s  = ESend;
    end else
      case (s)
        ESend:
          if (fall) begin
            if (tl > 0) begin
              coe = 1;
              co  = ts[47];
              ts  = ts << 1;
              tl  = tl - 1;
            end else begin
              coe = 0;
              co  = 1;
              to  = 0;
              rc  = 0;
              s   = t == tagged RespNone ? ETail : EWait;
            end
          end
        EWait:
          if (rise) begin
            if (cmdIn == 0) begin
              rs = 0;
              rc = 1;
              s  = ERecv;
            end else if (to >= 64) begin
              eTo = True;
              s   = EIdle;
            end else
              to = to + 1;
          end
        ERecv:
          if (rise) begin
            rs = {rs[134:0], cmdIn};
            rc = rc + 1;
            if (rc == respLen(t)) begin
              if (t == tagged R136) begin
                rp = parse136(rs);
                if (rs[0] == 0) eEnd = True;
                else done = True;
                s = EIdle;
              end else begin
                Resp48 p = parse48(r.cmd_index, rs[47:0]);
                eCrc = checksCrc(t) && !p.crcOk;
                eIdx = checksIndex(t) && !p.indexOk;
                eEnd = !p.endOk;
                rp   = zeroExtend(p.body);
                if (eCrc || eIdx || eEnd)
                  s = EIdle;
                else if (waitsBusy(t))
                  s = EBusy;
                else begin
                  done = True;
                  s    = EIdle;
                end
              end
            end
          end
        EBusy:
          if (rise && datIn[0] == 1) begin
            done = True;
            s    = EIdle;
          end
        ETail:
          if (rise) begin
            if (to >= 7) begin
              done = True;
              s    = EIdle;
            end else
              to = to + 1;
          end
      endcase

    // 数据通路：带数据的命令收完响应的那一拍接着起
    if (rstD[0]) begin
      dsl  = DNone;
      doe  = 0;
      dout = 1;
    end else begin
      if (done && r.cmd_data == 1 && dsl == DNone) begin
        wn = 0;
        nn = 0;
        dt = 0;
        if (r.cmd_read == 1)
          dsl = DWait;
        else begin
          dsl  = DFill;
          evWr = True;
        end
      end
      case (dsl)
        DFill:
          if (wrP[0]) begin
            bk = (bk << 32) | zeroExtend(swap(wrV[0]));
            wn = wn + 1;
            if (wn == 128) begin
              nn  = 0;
              dsl = DGap;
            end
          end
        DGap:
          if (rise) begin
            if (nn >= 1) begin
              nn   = 0;
              cr16 = 0;
              dsl  = DSend;
            end else
              nn = nn + 1;
          end
        // 起始位 0、4096 个数据位、16 位 CRC16、结束位 1，第 4114 个下降沿放开 DAT0
        DSend:
          if (fall) begin
            if (nn == 4114) begin
              doe  = 0;
              dout = 1;
              dt   = 0;
              tk   = 0;
              nn   = 0;
              dsl  = DTok;
            end else begin
              doe = 1;
              if (nn == 0)
                dout = 0;
              else if (nn <= 4096) begin
                dout = bk[4095];
                cr16 = crc16Step(cr16, bk[4095]);
                bk   = bk << 1;
              end else if (nn <= 4112) begin
                dout = cr16[15];
                cr16 = cr16 << 1;
              end else
                dout = 1;
              nn = nn + 1;
            end
          end
        // 令牌：起始位之后三位状态、一位结束位
        DTok:
          if (rise) begin
            if (nn == 0) begin
              if (datIn[0] == 0)
                nn = 1;
              else if (dt >= 64) begin
                eDto = True;
                dsl  = DNone;
              end else
                dt = dt + 1;
            end else begin
              tk = {tk[2:0], datIn[0]};
              nn = nn + 1;
              if (nn == 5) begin
                // 写错与认不出的令牌也按数据 CRC 错报（本版不细分）
                if (tokenOf(tk[3:1]) == Accepted)
                  dsl = DBusy;
                else begin
                  eDcrc = True;
                  dsl   = DNone;
                end
              end
            end
          end
        DBusy:
          if (rise && datIn[0] == 1) begin
            xfer = True;
            dsl  = DNone;
          end
        DWait:
          if (rise) begin
            if (datIn[0] == 0) begin
              nn   = 1;
              cr16 = 0;
              dsl  = DRecv;
            end else if (dt >= 1024) begin
              eDto = True;
              dsl  = DNone;
            end else
              dt = dt + 1;
          end
        DRecv:
          if (rise) begin
            if (nn <= 4096) begin
              bk   = {bk[4094:0], datIn[0]};
              cr16 = crc16Step(cr16, datIn[0]);
            end else if (nn <= 4112)
              crx = {crx[14:0], datIn[0]};
            else begin
              if (crx != cr16)
                eDcrc = True;
              else if (datIn[0] == 0)
                eDend = True;
              else begin
                evRd = True;
                wn   = 0;
              end
              dsl = evRd ? DDrain : DNone;
            end
            nn = nn + 1;
          end
        DDrain:
          if (rdP[0]) begin
            bk = bk << 32;
            wn = wn + 1;
            if (wn == 128) begin
              xfer = True;
              dsl  = DNone;
            end
          end
      endcase
    end

    st     <= s;
    rt     <= t;
    txSh   <= ts;
    txLeft <= tl;
    rxSh   <= rs;
    rxCnt  <= rc;
    tmo    <= to;
    resp   <= rp;
    cmdOr  <= co;
    cmdOeR <= coe;
    sdclk  <= ck;
    ckPrev <= sdclk;
    divCnt <= dc;
    evs    <= {b(eDend), b(eDcrc), b(eDto), b(evRd), b(evWr), b(xfer), b(eIdx), b(eEnd), b(eCrc), b(eTo), b(done)};
    ds     <= dsl;
    blk    <= bk;
    words  <= wn;
    dn     <= nn;
    c16    <= cr16;
    crcRx  <= crx;
    dtmo   <= dt;
    tok    <= tk;
    datOr  <= dout;
    datOeR <= doe;
    go[0]   <= False;
    rst[0]  <= False;
    rstD[0] <= False;
    rdP[0]  <= False;
    wrP[0]  <= False;
  endrule

  interface regs = r.regs;

  interface SdPins sd;
    method Bit#(1) clk = sdclk;
    method Bit#(1) cmdO = cmdOr;
    method Bit#(1) cmdOe = cmdOeR;
    method Bit#(4) datO = {3'b111, datOr};
    method Bit#(1) datOe = datOeR;
    method Action lines(Bit#(1) cmd, Bit#(4) dat);
      cmdIn <= cmd;
      datIn <= dat;
    endmethod
  endinterface

  method Bool irq = (isr & r.sigen) != 0;
endmodule

endpackage
