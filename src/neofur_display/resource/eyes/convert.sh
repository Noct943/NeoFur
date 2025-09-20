#!/bin/bash

# 定义原始文件名和目标尺寸
INPUT_FILE="green_left.png"
TARGET_SIZE="240x240"

# 使用 identify 获取调整尺寸后的图片信息
# ImageMagick 的 -format "%wx%h" 选项可以提取宽度和高度
# 注意这里我们用到了一个子 shell，将 resize 后的图片信息传递给 identify
# 这样做可以精确获取 resize 后的尺寸，而不用自己计算
NEW_SIZE=$(convert "$INPUT_FILE" -resize "$TARGET_SIZE" -format "%wx%h" info:)

# 生成新的文件名
# 假设你想在原文件名后加上尺寸信息，并改变后缀
NEW_FILE="${INPUT_FILE%.*}_${NEW_SIZE}.rgba"

# 执行 convert 命令
convert "$INPUT_FILE" -resize "$TARGET_SIZE" -alpha set -depth 8 RGBA:"$NEW_FILE"

# 打印最终生成的文件名
echo "图片已转换并保存为: $NEW_FILE"

