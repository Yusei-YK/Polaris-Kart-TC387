
rmdir Kart_Debug /s /q
rmdir .ads /s /q

rem 原来这里会递归清掉所有 .launch:改工程名后它们都指着旧名字,
rem 逐飞的改名脚本靠这一步重置。但现在两份调试配置是入库文件,
rem 不能删;真要改工程名,把那两个文件一起改名。

exit
