# Makefile — exp2_lights v2.0 独立编译 (不依赖 ROS2)
#
# 用法:
#   make              # 编译库 + 测试程序
#   sudo ./test_lights # 运行测试 (需要 root 权限操作 GPIO)
#   make clean        # 清理

CC       = gcc
CFLAGS   = -std=c11 -Wall -Wextra -O2
LDFLAGS  = -lgpiod

# 目标
LIB      = libexp2_lights.a
TEST     = test_lights

# 源文件
LIB_SRC  = lights.c
TEST_SRC = test_lights.c

# 目标文件
LIB_OBJ  = $(LIB_SRC:.c=.o)
TEST_OBJ = $(TEST_SRC:.c=.o)

.PHONY: all clean

all: $(LIB) $(TEST)

# 静态库
$(LIB): $(LIB_OBJ)
	ar rcs $@ $^

# 测试程序
$(TEST): $(TEST_OBJ) $(LIB)
	$(CC) $(CFLAGS) -o $@ $(TEST_OBJ) -L. -lexp2_lights $(LDFLAGS)

# 编译规则
%.o: %.c lights.h
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(LIB_OBJ) $(TEST_OBJ) $(LIB) $(TEST)
