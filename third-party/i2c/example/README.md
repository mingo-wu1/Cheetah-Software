编译：
gcc -o test main.c ../i2c.c -I ../

测试例子：
sudo ./test 8 0x10 1 8
