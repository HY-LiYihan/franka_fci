"""Service entrypoint."""

from __future__ import annotations

import uvicorn

from franka_fci.config import get_settings


def main() -> None:
    settings = get_settings()
    uvicorn.run("franka_fci.api.app:create_app", factory=True, host=settings.api_host, port=settings.api_port)


if __name__ == "__main__":
    main()
