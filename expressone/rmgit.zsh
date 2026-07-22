#!/usr/bin/env zsh
# rmgit.zsh — 清除本机 git 凭证

set -e

echo "=== 清除 git 凭证 ==="

# 1. 删除 credential 文件
if [[ -f ~/.git-credentials ]]; then
    echo "删除 ~/.git-credentials"
    rm ~/.git-credentials
else
    echo "~/.git-credentials 不存在，跳过"
fi

# 2. 取消 credential.helper
if git config --global credential.helper &>/dev/null; then
    echo "取消 git config credential.helper"
    git config --global --unset credential.helper
else
    echo "credential.helper 未设置，跳过"
fi

# 3. 检查残留
echo ""
echo "=== 检查残留 ==="
if git config --global --list 2>/dev/null | grep -qi cred; then
    echo "[!] 仍有 credential 相关配置:"
    git config --global --list | grep -i cred
else
    echo "[✓] 无残留 credential 配置"
fi

if [[ -f ~/.git-credentials ]]; then
    echo "[!] ~/.git-credentials 仍存在"
else
    echo "[✓] ~/.git-credentials 已清除"
fi

echo ""
echo "=== 完成 ==="
