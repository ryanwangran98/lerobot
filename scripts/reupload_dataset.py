#!/usr/bin/env python

from pathlib import Path
from huggingface_hub import HfApi
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# 配置
REPO_ID = "你的用户名/数据集名称"  # 修改为你的 dataset.repo_id
DATASET_ROOT = None  # 默认为 ~/.cache/huggingface/hub/lerobot/你的数据集名称
PRIVATE = False

def reupload_dataset(repo_id: str, root: str | Path | None = None, private: bool = False):
    """
    重新上传已录制的数据集到 Hugging Face Hub

    Args:
        repo_id: 数据集 repo_id，例如 "username/dataset_name"
        root: 数据集本地路径，默认为 ~/.cache/huggingface/hub/lerobot/<repo_id>
        private: 是否私有仓库
    """
    # 加载本地数据集
    dataset = LeRobotDataset(repo_id, root=root)

    print(f"准备上传数据集: {repo_id}")
    print(f"本地路径: {dataset.root}")
    print(f"总 episode 数: {dataset.num_episodes}")
    print(f"总帧数: {dataset.meta.total_frames}")
    print(f"视频键: {dataset.meta.video_keys}")

    # 上传到 Hub
    print("\n开始上传...")
    try:
        dataset.push_to_hub(
            private=private,
            push_videos=True,
        )
        print(f"✅ 上传成功！")
        print(f"数据集地址: https://huggingface.co/datasets/{repo_id}")
    except Exception as e:
        print(f"❌ 上传失败: {e}")
        raise

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="重新上传已录制的数据集到 Hugging Face Hub")
    parser.add_argument("--repo_id", type=str, required=True, help="数据集 repo_id，例如 'username/dataset_name'")
    parser.add_argument("--root", type=str, default=None, help="数据集本地路径，默认为缓存目录")
    parser.add_argument("--private", action="store_true", help="创建私有仓库")
    args = parser.parse_args()

    reupload_dataset(args.repo_id, args.root, args.private)
