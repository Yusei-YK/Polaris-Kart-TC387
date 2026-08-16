# -*- coding: utf-8 -*-
"""PreToolUse 守卫。exit 2 = 拦住这次调用,stderr 回喂给模型。

存在理由:CLAUDE.md 的散文规则被反复无视(第6行写了没工具链还是扫了三个目录,
第112行写了别先抛结论还是抛了三个)。只有机械拦截有效。
"""
import json
import re
import sys

try:
    ev = json.load(sys.stdin)
except Exception:
    sys.exit(0)

tool = ev.get("tool_name", "")
ti = ev.get("tool_input", {}) or {}


def die(msg):
    sys.stderr.write(msg + "\n")
    sys.exit(2)


def paths():
    out = []
    for k in ("file_path", "notebook_path", "path", "pattern"):
        v = ti.get(k)
        if isinstance(v, str):
            out.append(v)
    return out


# --- 1. 副本目录:Glob 会把它排在真目录前面,改了等于没改 -----------------
for p in paths():
    n = p.replace("\\", "/")
    if "Kart_TC387 2" in n or "SCC8660_Product-master" in n:
        die("BLOCKED: 这是未跟踪的副本目录,不是真源码。\n"
            "真路径: G:/CODE/Smart car/SmartCar/Kart_TC387/...\n"
            "改副本的后果: 静态校验全过、git 看不见、用户编译出来没效果。")

# --- 2. Edit/MultiEdit 在本仓库必失败(行尾混用) -------------------------
if tool in ("Edit", "MultiEdit"):
    die("BLOCKED: 本仓库行尾混用 + core.autocrlf=true,Edit/MultiEdit 必失败。\n"
        "唯一手段: 写 python 脚本到 C:/tmp/pNN_xxx.py,normalize 后运行。\n"
        "读写用 io.open(p,'rb') -> decode('utf-8').replace('\\r\\n','\\n'),"
        "写回 'wb' + encode('utf-8')。替换前 assert count==1。")

# --- 3. 脚本里手敲 \\uXXXX:一夜错两次(纠/瞄),每次都是 count=0 重来 -----
if tool == "Write":
    fp = (ti.get("file_path") or "").replace("\\", "/")
    body = ti.get("content") or ""
    if fp.endswith(".py") and re.search(r"\\u[0-9a-fA-F]{4}", body):
        die("BLOCKED: 脚本里出现手敲的 \\uXXXX。已经因此错过两次(纠写成\\u7eaa、"
            "瞄写成\\u77c4),每次都是 count=0 失败后重写重跑。\n"
            "做法: 先 Read 看到真字,把真字直接粘进锚点,文件存 UTF-8。")
    if fp.endswith(".py") and "assert" not in body and re.search(r"\.replace\(", body):
        die("BLOCKED: 替换脚本里没有 assert count==1。\n"
            "每个 replace 前必须 assert d.count(anchor)==1,否则改错位置无声通过。")

# --- 4. bash 层面的坑 ---------------------------------------------------
if tool == "Bash":
    cmd = ti.get("command") or ""
    low = cmd.lower()

    if re.search(r"git\s+add\s+(\.|-A|--all)(\s|$)", cmd):
        die("BLOCKED: git add . 会把 824 个副本文件入库。\n"
            "只用 git add -u (已跟踪文件的修改),或显式列文件名。")

    if re.search(r"\b(rm\s+-rf|git\s+reset\s+--hard|git\s+clean\s+-[a-z]*f|"
                 r"git\s+push\s+.*--force|git\s+branch\s+-D)\b", cmd):
        die("BLOCKED: 破坏性命令。先跟用户确认。")

    if re.search(r"\b(rg|grep|find|ls)\b", cmd) and "libraries/" in cmd:
        die("BLOCKED: 不扫 libraries/。逐飞库不改,扫它纯浪费时间。")

    if re.search(r"\b(rg|grep|find)\b", cmd) and "Kart_TC387 2" not in cmd:
        if not re.search(r"^\s*(python|git)\b", cmd):
            die("BLOCKED: 搜索走内置 Grep/Glob 工具,别走 bash。\n"
                "rg 在本机只是 shell 别名(PATH 上没有),find 会超时。")

    for kw in ("infineon", "tasking", "aurix", "tricore"):
        if kw in low and re.search(r"\b(ls|find|where|dir)\b", low):
            die("BLOCKED: 本机没有 TriCore 工具链,CLAUDE.md 第6行已经写了。\n"
                "已经因此扫过三个目录一次。不要重新发现已知事实。")

    if re.search(r"^\s*(cat|head|tail)\s+", cmd):
        die("BLOCKED: 读文件用 Read 工具,不用 cat/head/tail。")

    if "$(" in cmd and re.search(r"\bfor\b.*\bdo\b", cmd):
        die("BLOCKED: 双引号里的 $(...) 在循环里不展开,会把字面量打出来(已踩过)。\n"
            "路径带空格 + 循环变量的组合改用 python 一次算完。")

sys.exit(0)
