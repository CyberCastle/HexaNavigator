from __future__ import annotations

import logging
import logging.handlers
import os
from typing import Optional


def setup_logging(level: str = "INFO", to_file: bool = True, log_dir: Optional[str] = None) -> None:
    """Configure root logger with console and optional RotatingFileHandler.

    Args:
        level: Log level name (e.g., "DEBUG", "INFO").
        to_file: If True, write logs to logs/app.log with rotation.
        log_dir: Directory for log files; defaults to ./logs.
    """
    lvl = getattr(logging, level.upper(), logging.INFO)
    logger = logging.getLogger()
    logger.setLevel(lvl)

    # Clear existing handlers to avoid duplicates
    for h in list(logger.handlers):
        logger.removeHandler(h)

    fmt = logging.Formatter(
        fmt="%(asctime)s %(levelname)s [%(name)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    ch = logging.StreamHandler()
    ch.setLevel(lvl)
    ch.setFormatter(fmt)
    logger.addHandler(ch)

    if to_file:
        log_dir = log_dir or os.path.join(os.getcwd(), "logs")
        os.makedirs(log_dir, exist_ok=True)
        fh = logging.handlers.RotatingFileHandler(
            os.path.join(log_dir, "app.log"), maxBytes=2 * 1024 * 1024, backupCount=3
        )
        fh.setLevel(lvl)
        fh.setFormatter(fmt)
        logger.addHandler(fh)
