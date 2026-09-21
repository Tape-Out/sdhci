"""sdhci 的行为测试台：命令通路与单块 PIO 数据通路。判据见 notes/规范对照/sdhci.md「判据（先写）」与「第二刀」。

测试台经控制口驱主机控制器，SD 卡是测试台里的行为模型（照 litesdcard 卡模拟器 sd_link.v 的状态与命令，自己写，不拷代码）：
卡在 SD 时钟上升沿采 CMD、DAT0，出下一位；收齐 48 位先查 CRC7 与结束位，对了才回应，隔两个时钟起发。
卡模型的 CRC7、CRC16 都逐位移位另写一份，不用被测件的 Gf2；期望的 CRC、CID 分段、图样的字节和由这里的 Python 另算。
卡记下的是主机帧里发来的 CRC 字段，不是自己重算的值（变异 crcpoly 逼出来的）。

一，版本 3.00、能力报 3.3 V；软复位全部读回零；内部时钟使能后报稳定。
二，CMD0：发命令期间命令禁止位为 1；完成中断置位、中断线拉高；主机发出的 CRC7 对；W1C 清掉后中断线落下。
三，CMD8：响应回显 0x1AA，没有错误；主机发出的 CRC7 对。
四，CMD55 接 ACMD41 轮询到就绪，每次都完成且无错（R3 不查 CRC），第三次才就绪，OCR 带高容量位。
五，CMD2 的 136 位响应：CID 去掉 CRC 那一字节，右对齐落进四个响应字。CMD3 拿到 RCA。CMD7 带忙：卡放开 DAT0 之后才报完成。
六，CMD24 写第 5 块：完成与可写中断，状态位；写 128 个字后传输完成，完成时卡已放开 DAT0 的忙；卡收下，主机发的 CRC16 对，卡里的字节和、首尾字节对。
七，CMD17 读第 5 块：完成与可读中断，状态位；读回的 128 个字与写进的相同；读完第 127 个字时还没完成，读完最后一个字才完成。
八，数据错：卡把读块的 CRC16 错一位，报数据 CRC 错、不置可读；卡回 CRC 错令牌，报数据 CRC 错、不置传输完成，卡里的块不变。
九，命令错：卡不回应的命令报超时；CRC7 错、命令号错、结束位错各报各的；命令不要求查 CRC 时 CRC 错不算错。
"""
import json
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")


def crc_bits(bits, width, poly):
    c = 0
    top = 1 << (width - 1)
    mask = (1 << width) - 1
    for b in bits:
        fb = (1 if c & top else 0) ^ b
        c = ((c << 1) & mask) ^ (poly if fb else 0)
    return c


def cmd_crc(idx, arg):
    # 起始位 0、传输位 1、6 位命令号、32 位参数，高位先
    body = (1 << 38) | (idx << 32) | arg
    return crc_bits([(body >> (39 - i)) & 1 for i in range(40)], 7, 0x09)


CID = 0x0353_4453_4330_3447_8012_3456_7801_4A01
content = CID >> 8                       # 去掉 CRC 与结束位那一字节，剩 120 位
cid_words = [(content >> (32 * i)) & 0xFFFF_FFFF for i in range(4)]

# 写进第 5 块的图样：第 i 个字，小端落成四个字节
words = [((i * 0x0101_0101) ^ 0xA5C3_0F96) & 0xFFFF_FFFF for i in range(128)]
data = b"".join(w.to_bytes(4, "little") for w in words)
SUM = sum(data) & 0xFFFF_FFFF
CRC16 = crc_bits([(byte >> (7 - k)) & 1 for byte in data for k in range(8)], 16, 0x1021)

verdict = ("the host controller initialises the card with correct CRC7, lands 48-bit and 136-bit responses, waits out "
           "R1b busy, writes and reads back a 512-byte block over PIO with correct CRC16, and reports command and "
           "data timeout, CRC, index and end bit errors only when they apply")

TEMPLATE = r'''package Sdhci@L@Tb;

// 由 htest/mksdhcitb.py 生成，勿手改

import StmtFSM::*;
import RegIf::*;
import Sdhci::*;

typedef struct {
  UInt#(8)  len;     // 0 表示不回应
  Bit#(136) frame;   // 左对齐，高位先发
  UInt#(8)  busy;    // 回应发完之后 DAT0 拉低几个时钟
} CardRsp deriving (Bits);

(* synthesize *)
module mkSdhci@L@Tb(Empty);
  SdhciIfc#(8, 32) d <- mkSdhci(SdhciCfg { none: ? });

  // ---------------- 卡模型：CMD 线 ----------------
  Reg#(Bit#(1))   ckPrev   <- mkReg(0);
  Reg#(Bool)      rxOn     <- mkReg(False);
  Reg#(UInt#(6))  rxCnt    <- mkReg(0);
  Reg#(Bit#(48))  rxSh     <- mkReg(0);
  Reg#(Bit#(7))   rxCrc    <- mkReg(0);
  Reg#(Bit#(136)) txSh     <- mkReg(0);
  Reg#(UInt#(8))  txLeft   <- mkReg(0);
  Reg#(UInt#(4))  txWait   <- mkReg(0);
  Reg#(CardRsp)   pend     <- mkReg(CardRsp { len: 0, frame: 0, busy: 0 });
  Reg#(UInt#(8))  busyLeft <- mkReg(0);
  Reg#(Bit#(1))   cardCmd  <- mkReg(1);
  Reg#(Bool)      appCmd   <- mkReg(False);
  Reg#(UInt#(4))  tries41  <- mkReg(0);
  Reg#(Bit#(4))   cst      <- mkReg(0);
  Reg#(Bit#(3))   inj      <- mkReg(0);
  Reg#(Bit#(7))   lastCrc  <- mkReg(0);
  Reg#(UInt#(16)) edges    <- mkReg(0);
  RWire#(Bit#(3)) injW     <- mkRWire;

  // ---------------- 卡模型：DAT0 ----------------
  Reg#(Bit#(4096)) store     <- mkReg(0);        // 第 5 块
  Reg#(UInt#(2))   dPh       <- mkReg(0);        // 0 不发，1 发数据块，2 发令牌
  Reg#(UInt#(2))   dNext     <- mkReg(0);
  Reg#(UInt#(13))  dCnt      <- mkReg(0);
  Reg#(Bit#(4096)) dSh       <- mkReg(0);
  Reg#(Bit#(16))   dCrc      <- mkReg(0);
  Reg#(Bit#(4))    dTok      <- mkReg(0);
  Reg#(UInt#(4))   dWait     <- mkReg(0);
  Reg#(UInt#(8))   dBusyN    <- mkReg(0);
  Reg#(Bool)       dArmBlk   <- mkReg(False);    // CMD17 的响应发完之后发块
  Reg#(Bool)       dBad      <- mkReg(False);
  Reg#(Bool)       dRxWant   <- mkReg(False);    // CMD24 收下了，响应还没发完
  Reg#(Bool)       dRxArm    <- mkReg(False);    // 等主机的数据块
  Reg#(Bool)       dRxOn     <- mkReg(False);
  Reg#(UInt#(13))  dRxCnt    <- mkReg(0);
  Reg#(Bit#(4096)) dRx       <- mkReg(0);
  Reg#(Bit#(16))   dRxCrc    <- mkReg(0);
  Reg#(Bit#(16))   dRxField  <- mkReg(0);
  Reg#(Bool)       dRefuse   <- mkReg(False);
  Reg#(Bit#(1))    cardDat   <- mkReg(1);
  Reg#(UInt#(16))  writes    <- mkReg(0);
  Reg#(Bit#(16))   lastCrc16 <- mkReg(0);

  Bit#(1) cmdLine = (d.sd.cmdOe == 1 ? d.sd.cmdO : 1) & cardCmd;
  Bit#(1) cardLine = cardDat & (busyLeft > 0 ? 1'b0 : 1'b1);
  Bit#(4) datLine  = (d.sd.datOe == 1 ? d.sd.datO : 4'hF) & {3'b111, cardLine};

  function Bit#(7) crc7Step(Bit#(7) c, Bit#(1) b);
    Bit#(1) fb = c[6] ^ b;
    return {c[5:0], 1'b0} ^ (fb == 1 ? 7'h09 : 0);
  endfunction

  function Bit#(16) crc16Step(Bit#(16) c, Bit#(1) b);
    Bit#(1) fb = c[15] ^ b;
    return {c[14:0], 1'b0} ^ (fb == 1 ? 16'h1021 : 0);
  endfunction

  function Bit#(7) crc7Ser(Bit#(40) x);
    Bit#(7) c = 0;
    for (Integer i = 39; i >= 0; i = i - 1) c = crc7Step(c, x[i]);
    return c;
  endfunction

  function Bit#(32) sum512(Bit#(4096) x);
    Bit#(32) sm = 0;
    for (Integer i = 0; i < 512; i = i + 1) begin
      Bit#(8) byt = x[i * 8 + 7:i * 8];
      sm = sm + zeroExtend(byt);
    end
    return sm;
  endfunction

  // bad：1 错一位 CRC，2 命令号错（CRC 按错的号算，只错命令号），3 结束位为 0
  function Bit#(136) r48(Bit#(6) idx, Bit#(32) body, Bit#(2) bad);
    Bit#(6)  i2 = bad == 2 ? idx ^ 1 : idx;
    Bit#(40) h  = {2'b00, i2, body};
    Bit#(7)  c  = crc7Ser(h) ^ (bad == 1 ? 7'h01 : 0);
    Bit#(1)  e  = bad == 3 ? 0 : 1;
    return {h, c, e, 88'b0};
  endfunction

  // R1 卡状态：12:9 当前状态，8 可收数据，5 下一条是应用命令
  function Bit#(32) r1(Bit#(4) s, Bool app) = zeroExtend({s, 1'b1, 2'b00, app ? 1'b1 : 1'b0, 5'b00000});

  Bit#(128) cid = 128'h@CID@;

  function CardRsp answer(Bit#(6) idx, Bit#(32) arg, Bool app, UInt#(4) t41, Bit#(4) s, Bit#(2) bad);
    CardRsp none = CardRsp { len: 0, frame: 0, busy: 0 };
    CardRsp a = none;
    case (idx)
      8:  a = CardRsp { len: 48, frame: r48(8, zeroExtend(arg[11:0]), bad), busy: 0 };
      55: a = CardRsp { len: 48, frame: r48(55, r1(s, True), bad), busy: 0 };
      41: if (app) a = CardRsp { len: 48, frame: {2'b00, 6'b111111, t41 >= 2 ? 32'hC0FF_8000 : 32'h40FF_8000, 7'h7F, 1'b1, 88'b0}, busy: 0 };
      2:  a = CardRsp { len: 136, frame: {2'b00, 6'b111111, cid[127:1], 1'b1}, busy: 0 };
      3:  a = CardRsp { len: 48, frame: r48(3, {16'h1234, 16'h0500}, bad), busy: 0 };
      7:  if (arg[31:16] == 16'h1234) a = CardRsp { len: 48, frame: r48(7, r1(s, False), bad), busy: 40 };
      13: a = CardRsp { len: 48, frame: r48(13, r1(s, False), bad), busy: 0 };
      17: if (arg == 5) a = CardRsp { len: 48, frame: r48(17, r1(s, False), bad), busy: 0 };
      24: if (arg == 5) a = CardRsp { len: 48, frame: r48(24, r1(s, False), bad), busy: 0 };
    endcase
    return a;
  endfunction

  rule card;
    Bit#(1)    ck   = d.sd.clk;
    Bit#(3)    ij   = inj;
    Bool       on   = rxOn;
    UInt#(6)   n    = rxCnt;
    Bit#(48)   sh   = rxSh;
    Bit#(7)    cr   = rxCrc;
    Bit#(136)  tx   = txSh;
    UInt#(8)   tl   = txLeft;
    UInt#(4)   tw   = txWait;
    CardRsp    pd   = pend;
    UInt#(8)   bl   = busyLeft;
    Bit#(1)    cc   = cardCmd;
    Bool       ap   = appCmd;
    UInt#(4)   t41  = tries41;
    Bit#(4)    s    = cst;
    Bit#(7)    lc   = lastCrc;
    UInt#(16)  ed   = edges;
    Bit#(4096) st5  = store;
    UInt#(2)   ph   = dPh;
    UInt#(2)   nx   = dNext;
    UInt#(13)  dc   = dCnt;
    Bit#(4096) dsh  = dSh;
    Bit#(16)   dcr  = dCrc;
    Bit#(4)    tkn  = dTok;
    UInt#(4)   dwt  = dWait;
    UInt#(8)   bn   = dBusyN;
    Bool       armB = dArmBlk;
    Bool       bad4 = dBad;
    Bool       rxW  = dRxWant;
    Bool       rxA  = dRxArm;
    Bool       rxO  = dRxOn;
    UInt#(13)  rc2  = dRxCnt;
    Bit#(4096) rx2  = dRx;
    Bit#(16)   rcrc = dRxCrc;
    Bit#(16)   rfld = dRxField;
    Bool       refu = dRefuse;
    Bit#(1)    cd   = cardDat;
    UInt#(16)  nw   = writes;
    Bit#(16)   l16  = lastCrc16;
    if (injW.wget matches tagged Valid .k) ij = k;
    if (ck == 1 && ckPrev == 0) begin
      ed = ed + 1;
      // CMD 线
      if (tl > 0) begin
        cc = tx[135];
        tx = tx << 1;
        tl = tl - 1;
        if (tl == 0) begin
          bl = pd.busy;
          if (armB) begin
            armB = False;
            nx   = 1;
            dwt  = 2;
          end
          if (rxW) begin
            rxW = False;
            rxA = True;
          end
        end
      end else begin
        cc = 1;
        if (bl > 0) bl = bl - 1;
        if (tw > 0) begin
          tw = tw - 1;
          if (tw == 0) begin
            tx = pd.frame;
            tl = pd.len;
          end
        end else if (!on) begin
          if (cmdLine == 0 && d.sd.cmdOe == 1) begin
            on = True;
            n  = 1;
            sh = 0;
            cr = crc7Step(0, 0);
          end
        end else begin
          sh = {sh[46:0], cmdLine};
          if (n < 40) cr = crc7Step(cr, cmdLine);
          n = n + 1;
          if (n == 48) begin
            on = False;
            Bit#(6)  idx = sh[45:40];
            Bit#(32) arg = sh[39:8];
            lc = sh[7:1];
            if (sh[7:1] == cr && sh[0] == 1) begin
              Bool    wasApp = ap;
              Bit#(2) rb     = ij <= 3 ? truncate(ij) : 0;
              CardRsp a      = answer(idx, arg, wasApp, t41, s, rb);
              if (a.len > 0) begin
                pd = a;
                tw = 2;
                if (ij <= 3) ij = 0;
              end
              if (idx == 17 && arg == 5) begin
                armB = True;
                dsh  = st5;
                bad4 = ij == 4;
                if (ij == 4) ij = 0;
              end
              if (idx == 24 && arg == 5) begin
                rxW  = True;
                refu = ij == 5;
                if (ij == 5) ij = 0;
              end
              ap = idx == 55;
              if (idx == 0) begin
                t41 = 0;
                s   = 0;
              end else if (idx == 41 && wasApp) begin
                if (t41 >= 2) s = 1;
                t41 = t41 + 1;
              end else if (idx == 2)
                s = 2;
              else if (idx == 3)
                s = 3;
              else if (idx == 7 && arg[31:16] == 16'h1234)
                s = 4;
            end
          end
        end
      end

      // DAT0：卡发数据块（起始位、4096 位、CRC16、结束位）或令牌（起始位、三位状态、结束位）
      if (ph == 1) begin
        if (dc == 0) begin
          cd  = 0;
          dcr = 0;
        end else if (dc <= 4096) begin
          cd  = dsh[4095];
          dcr = crc16Step(dcr, dsh[4095]);
          dsh = dsh << 1;
        end else if (dc <= 4112) begin
          Bit#(16) c2 = (dc == 4097 && bad4) ? dcr ^ 16'h0001 : dcr;
          cd  = c2[15];
          dcr = c2 << 1;
        end else if (dc == 4113)
          cd = 1;
        else begin
          cd = 1;
          ph = 0;
        end
        dc = dc + 1;
      end else if (ph == 2) begin
        if (dc == 0)
          cd = 0;
        else if (dc <= 4) begin
          cd  = tkn[3];
          tkn = tkn << 1;
        end else begin
          cd = 1;
          ph = 0;
          bl = bn;
        end
        dc = dc + 1;
      end else begin
        cd = 1;
        if (dwt > 0) begin
          dwt = dwt - 1;
          if (dwt == 0) begin
            ph = nx;
            dc = 0;
          end
        end
      end

      // DAT0：收主机的数据块，CRC 对了才收下，令牌之后拉忙
      if (rxA && !rxO) begin
        if (d.sd.datOe == 1 && d.sd.datO[0] == 0) begin
          rxO  = True;
          rc2  = 1;
          rcrc = 0;
        end
      end else if (rxO) begin
        Bit#(1) hb = d.sd.datO[0];
        if (rc2 <= 4096) begin
          rx2  = {rx2[4094:0], hb};
          rcrc = crc16Step(rcrc, hb);
        end else if (rc2 <= 4112)
          rfld = {rfld[14:0], hb};
        else begin
          rxO = False;
          rxA = False;
          l16 = rfld;
          Bool ok = rfld == rcrc && hb == 1 && !refu;
          if (ok) begin
            st5 = rx2;
            nw  = nw + 1;
          end
          tkn = ok ? 4'b0101 : 4'b1011;
          bn  = ok ? 20 : 0;
          nx  = 2;
          dwt = 2;
        end
        rc2 = rc2 + 1;
      end
    end
    ckPrev    <= ck;
    inj       <= ij;
    rxOn      <= on;
    rxCnt     <= n;
    rxSh      <= sh;
    rxCrc     <= cr;
    txSh      <= tx;
    txLeft    <= tl;
    txWait    <= tw;
    pend      <= pd;
    busyLeft  <= bl;
    cardCmd   <= cc;
    appCmd    <= ap;
    tries41   <= t41;
    cst       <= s;
    lastCrc   <= lc;
    edges     <= ed;
    store     <= st5;
    dPh       <= ph;
    dNext     <= nx;
    dCnt      <= dc;
    dSh       <= dsh;
    dCrc      <= dcr;
    dTok      <= tkn;
    dWait     <= dwt;
    dBusyN    <= bn;
    dArmBlk   <= armB;
    dBad      <= bad4;
    dRxWant   <= rxW;
    dRxArm    <= rxA;
    dRxOn     <= rxO;
    dRxCnt    <= rc2;
    dRx       <= rx2;
    dRxCrc    <= rcrc;
    dRxField  <= rfld;
    dRefuse   <= refu;
    cardDat   <= cd;
    writes    <= nw;
    lastCrc16 <= l16;
  endrule

  rule lines;
    d.sd.lines(cmdLine, datLine);
  endrule

  // ---------------- 主机一侧的命令序列 ----------------
  Reg#(Bit#(32))  rv    <- mkReg(0);
  Reg#(Bool)      bad   <- mkReg(False);
  Reg#(UInt#(16)) tries <- mkReg(0);
  Reg#(UInt#(4))  loops <- mkReg(0);
  Reg#(Bit#(32))  i     <- mkReg(0);
  Reg#(UInt#(8))  mism  <- mkReg(0);
  Reg#(Bit#(32))  cyc   <- mkReg(0);

  function Action wr(Bit#(8) a, Bit#(32) v) = action
    let x <- d.regs.access(RegReq { addr: a, write: True, wdata: v, wstrb: 4'hF });
  endaction;
  function Action rd(Bit#(8) a) = action
    let x <- d.regs.access(RegReq { addr: a, write: False, wdata: 0, wstrb: 4'hF });
    rv <= x.rdata;
  endaction;
  function Action chk(String what, Bit#(32) got, Bit#(32) want) = action
    if (got != want) begin
      $display("FAIL %s: got %08h want %08h", what, got, want);
      bad <= True;
    end
  endaction;
  // 命令寄存器那个字：命令号 29:24，带数据 21，查命令号 20，查 CRC 19，响应类型 17:16；传输方式在低半字，读是第 4 位
  function Bit#(32) cmdWord(Bit#(6) idx, Bit#(2) resp, Bool crc, Bool ix) =
    (zeroExtend(idx) << 24) | (ix ? 32'h0010_0000 : 0) | (crc ? 32'h0008_0000 : 0) | (zeroExtend(resp) << 16);
  function Bit#(32) pat(Bit#(32) k) = (k * 32'h0101_0101) ^ 32'hA5C3_0F96;
  function Stmt waitInt(Bit#(32) mask) = seq
    tries <= 0;
    rd(8'h30);
    while ((rv & mask) == 0 && tries < 60000) action
      rd(8'h30);
      tries <= tries + 1;
    endaction
  endseq;

  Stmt test = seq
    // 一，版本、能力、软复位、内部时钟
    rd(8'hFC);  chk("host version is 3.00", (rv >> 16) & 32'hFF, 2);
    rd(8'h40);  chk("capabilities report 3.3 V", (rv >> 24) & 1, 1);
    wr(8'h2C, 32'h0100_0000);
    rd(8'h2C);  chk("the software reset bits read back zero", (rv >> 24) & 7, 0);
    wr(8'h2C, 32'h0000_0101);
    tries <= 0;
    rd(8'h2C);
    while ((rv & 2) == 0 && tries < 100) action
      rd(8'h2C);
      tries <= tries + 1;
    endaction
    chk("the internal clock reports stable", (rv >> 1) & 1, 1);
    wr(8'h2C, 32'h0000_0105);
    wr(8'h28, 32'h0000_0F00);
    wr(8'h34, 32'hFFFF_FFFF);
    wr(8'h38, 32'h0000_0001);

    // 二，CMD0
    wr(8'h08, 0);
    wr(8'h0C, cmdWord(0, 0, False, False));
    rd(8'h24);  chk("command inhibit is set while CMD0 is sent", rv & 1, 1);
    waitInt(1);
    chk("CMD0 completes", rv & 1, 1);
    chk("the host sends CMD0 with the right CRC7", zeroExtend(lastCrc), @CRC0@);
    chk("the card sees edges on the SD clock", zeroExtend(pack(edges > 0)), 1);
    chk("CMD0 raises the interrupt line", zeroExtend(pack(d.irq)), 1);
    wr(8'h30, 32'hFFFF_FFFF);
    rd(8'h30);  chk("writing ones clears the interrupt status", rv, 0);
    chk("clearing the status drops the interrupt line", zeroExtend(pack(d.irq)), 0);
    rd(8'h24);  chk("command inhibit clears after CMD0", rv & 1, 0);

    // 三，CMD8
    wr(8'h08, 32'h0000_01AA);
    wr(8'h0C, cmdWord(8, 2, True, True));
    waitInt(32'h0000_8001);
    chk("CMD8 completes without an error", rv & 32'hFFFF_8001, 1);
    chk("the host sends CMD8 with the right CRC7", zeroExtend(lastCrc), @CRC8@);
    rd(8'h10);  chk("CMD8 echoes the voltage and the check pattern", rv, 32'h0000_01AA);
    wr(8'h30, 32'hFFFF_FFFF);

    // 四，ACMD41 轮询到就绪
    loops <= 0;
    rv <= 0;
    while ((rv & 32'h8000_0000) == 0 && loops < 6) seq
      wr(8'h08, 0);
      wr(8'h0C, cmdWord(55, 2, True, True));
      waitInt(32'h0000_8001);
      wr(8'h30, 32'hFFFF_FFFF);
      wr(8'h08, 32'h40FF_8000);
      wr(8'h0C, cmdWord(41, 2, False, False));
      waitInt(32'h0000_8001);
      chk("ACMD41 completes without an error", rv & 32'hFFFF_8001, 1);
      wr(8'h30, 32'hFFFF_FFFF);
      rd(8'h10);
      loops <= loops + 1;
    endseq
    chk("ACMD41 reports the card ready with high capacity", rv & 32'hC000_0000, 32'hC000_0000);
    chk("the card becomes ready on the third ACMD41", zeroExtend(pack(loops)), 3);

    // 五，CMD2、CMD3、CMD7
    wr(8'h08, 0);
    wr(8'h0C, cmdWord(2, 1, True, False));
    waitInt(32'h0000_8001);
    chk("CMD2 completes without an error", rv & 32'hFFFF_8001, 1);
    rd(8'h10);  chk("CID bits 39..8 land in response word 0", rv, @CID0@);
    rd(8'h14);  chk("CID bits 71..40 land in response word 1", rv, @CID1@);
    rd(8'h18);  chk("CID bits 103..72 land in response word 2", rv, @CID2@);
    rd(8'h1C);  chk("CID bits 127..104 land in response word 3", rv, @CID3@);
    wr(8'h30, 32'hFFFF_FFFF);
    wr(8'h0C, cmdWord(3, 2, True, True));
    waitInt(32'h0000_8001);
    chk("CMD3 completes without an error", rv & 32'hFFFF_8001, 1);
    rd(8'h10);  chk("CMD3 returns the relative card address", rv >> 16, 32'h1234);
    wr(8'h30, 32'hFFFF_FFFF);
    wr(8'h08, 32'h1234_0000);
    wr(8'h0C, cmdWord(7, 3, True, True));
    waitInt(32'h0000_8001);
    chk("CMD7 completes without an error", rv & 32'hFFFF_8001, 1);
    chk("CMD7 completes only after the card releases DAT0", zeroExtend(pack(busyLeft)), 0);
    wr(8'h30, 32'hFFFF_FFFF);

    // 六，CMD24 写第 5 块
    wr(8'h08, 5);
    wr(8'h0C, cmdWord(24, 2, True, True) | 32'h0020_0000);
    waitInt(32'h0000_8010);
    chk("CMD24 completes and raises buffer write ready", rv & 32'hFFFF_8011, 32'h0000_0011);
    rd(8'h24);  chk("present state shows data inhibit, write active and buffer write enable", rv & 32'h0000_0F02, 32'h0000_0502);
    wr(8'h30, 32'hFFFF_FFFF);
    for (i <= 0; i < 128; i <= i + 1) wr(8'h20, pat(i));
    waitInt(32'h0000_8002);
    chk("the block write completes without an error", rv & 32'hFFFF_8002, 32'h0000_0002);
    chk("the block write completes only after the card releases DAT0", zeroExtend(pack(busyLeft)), 0);
    chk("the card takes the block", zeroExtend(pack(writes)), 1);
    chk("the host sends the block with the right CRC16", zeroExtend(lastCrc16), @CRC16@);
    chk("the card's block sums to the bytes written", sum512(store), @SUM@);
    chk("the card's first byte is the first byte written", zeroExtend(store[4095:4088]), @B0@);
    chk("the card's last byte is the last byte written", zeroExtend(store[7:0]), @B511@);
    wr(8'h30, 32'hFFFF_FFFF);
    rd(8'h24);  chk("data inhibit clears after the write", rv & 32'h0000_0F02, 0);

    // 七，CMD17 读第 5 块
    wr(8'h0C, cmdWord(17, 2, True, True) | 32'h0020_0010);
    waitInt(32'h0000_8020);
    chk("CMD17 completes and raises buffer read ready", rv & 32'hFFFF_8021, 32'h0000_0021);
    rd(8'h24);  chk("present state shows data inhibit, read active and buffer read enable", rv & 32'h0000_0F02, 32'h0000_0A02);
    wr(8'h30, 32'hFFFF_FFFF);
    mism <= 0;
    for (i <= 0; i < 128; i <= i + 1) seq
      rd(8'h20);
      action
        if (rv != pat(i)) mism <= mism + 1;
      endaction
      if (i == 126) seq
        rd(8'h30);
        chk("the read is not complete before the last word", rv & 2, 0);
      endseq
    endseq
    chk("the block reads back word for word", zeroExtend(pack(mism)), 0);
    waitInt(32'h0000_8002);
    chk("reading the last word completes the transfer", rv & 32'hFFFF_8002, 32'h0000_0002);
    wr(8'h30, 32'hFFFF_FFFF);

    // 八，数据错
    action injW.wset(4); endaction
    wr(8'h0C, cmdWord(17, 2, True, True) | 32'h0020_0010);
    waitInt(32'h0000_8020);
    chk("a read block with a bad CRC16 raises the data CRC error and no read ready", rv & 32'hFFFF_8020, 32'h0020_8000);
    wr(8'h30, 32'hFFFF_FFFF);
    action injW.wset(5); endaction
    wr(8'h0C, cmdWord(24, 2, True, True) | 32'h0020_0000);
    waitInt(32'h0000_8010);
    wr(8'h30, 32'hFFFF_FFFF);
    for (i <= 0; i < 128; i <= i + 1) wr(8'h20, ~pat(i));
    waitInt(32'h0000_8002);
    chk("a CRC error token raises the data CRC error and no transfer complete", rv & 32'hFFFF_8002, 32'h0020_8000);
    chk("the card keeps the old block after refusing a write", sum512(store), @SUM@);
    wr(8'h30, 32'hFFFF_FFFF);

    // 九，命令错
    wr(8'h08, 0);
    wr(8'h0C, cmdWord(5, 2, True, True));
    waitInt(32'h0001_0001);
    chk("a command the card ignores times out", rv & 32'h000F_8001, 32'h0001_8000);
    wr(8'h30, 32'hFFFF_FFFF);
    rd(8'h30);  chk("the error status clears", rv, 0);
    action injW.wset(1); endaction
    wr(8'h08, 32'h1234_0000);
    wr(8'h0C, cmdWord(13, 2, True, True));
    waitInt(32'h0000_8001);
    chk("a response with a bad CRC7 raises the command CRC error", rv & 32'h000F_8001, 32'h0002_8000);
    wr(8'h30, 32'hFFFF_FFFF);
    action injW.wset(2); endaction
    wr(8'h0C, cmdWord(13, 2, True, True));
    waitInt(32'h0000_8001);
    chk("a response with the wrong index raises the index error", rv & 32'h000F_8001, 32'h0008_8000);
    wr(8'h30, 32'hFFFF_FFFF);
    action injW.wset(3); endaction
    wr(8'h0C, cmdWord(13, 2, True, True));
    waitInt(32'h0000_8001);
    chk("a response without its end bit raises the end bit error", rv & 32'h000F_8001, 32'h0004_8000);
    wr(8'h30, 32'hFFFF_FFFF);
    action injW.wset(1); endaction
    wr(8'h0C, cmdWord(13, 2, False, True));
    waitInt(32'h0000_8001);
    chk("a bad CRC7 is no error when the command does not ask for the check", rv & 32'h000F_8001, 1);
    wr(8'h30, 32'hFFFF_FFFF);
  endseq;

  FSM fsm <- mkFSM(test);
  Reg#(Bool) started <- mkReg(False);

  rule go (!started);
    started <= True;
    fsm.start;
  endrule

  rule count;
    cyc <= cyc + 1;
    if (cyc > 3000000) begin
      $display("TIMEOUT");
      $finish(1);
    end
  endrule

  rule fin (started && fsm.done);
    if (bad) $display("FAILED");
    else $display("PASS sdhci: @VERDICT@");
    $finish(bad ? 1 : 0);
  endrule
endmodule

endpackage
'''

text = (TEMPLATE.replace("@L@", label)
        .replace("@CID@", f"{CID:032X}")
        .replace("@CRC0@", f"32'h{cmd_crc(0, 0):02X}")
        .replace("@CRC8@", f"32'h{cmd_crc(8, 0x1AA):02X}")
        .replace("@CID0@", f"32'h{cid_words[0]:08X}")
        .replace("@CID1@", f"32'h{cid_words[1]:08X}")
        .replace("@CID2@", f"32'h{cid_words[2]:08X}")
        .replace("@CID3@", f"32'h{cid_words[3]:08X}")
        .replace("@SUM@", f"32'h{SUM:08X}")
        .replace("@B0@", f"32'h{data[0]:02X}")
        .replace("@B511@", f"32'h{data[511]:02X}")
        .replace("@CRC16@", f"32'h{CRC16:04X}")
        .replace("@VERDICT@", verdict))
(out / f"Sdhci{label}Tb.bsv").write_text(text, encoding="utf-8")
print(f"  sdhci 行为测试台就位：标签 {label or '（空）'}；CMD0 的 CRC7 {cmd_crc(0, 0):02X}，CMD8(0x1AA) 的 {cmd_crc(8, 0x1AA):02X}，"
      f"块的 CRC16 {CRC16:04X}、字节和 {SUM:08X}")
