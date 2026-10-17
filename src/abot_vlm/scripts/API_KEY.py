# -*- coding: utf-8 -*-
"""不要把真实密钥写入共享工程；运行时使用环境变量 VLM_API_KEY。"""
import os

YI_KEY = os.environ.get("VLM_API_KEY", "")
