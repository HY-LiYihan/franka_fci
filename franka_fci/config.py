"""Application configuration loaded from environment variables."""

from __future__ import annotations

from functools import lru_cache
from typing import Any

from pydantic import Field, field_validator, model_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
        case_sensitive=False,
        env_prefix="FRANKA_",
    )

    robot_ip: str = "172.16.0.2"
    robot_model: str = "fr3"
    gripper_enabled: bool = True
    gripper_flange_translation_m: tuple[float, float, float] | None = None
    gripper_flange_quaternion_xyzw: tuple[float, float, float, float] | None = None

    api_host: str = "127.0.0.1"
    api_port: int = Field(default=8000, ge=1, le=65535)
    state_stale_after_ms: int = Field(default=250, ge=1)
    max_motion_duration_s: float = Field(default=30.0, gt=0.0)
    allowed_origins: str = ""

    @model_validator(mode="after")
    def validate_safety_configuration(self) -> "Settings":
        if (self.gripper_flange_translation_m is None) != (
            self.gripper_flange_quaternion_xyzw is None
        ):
            raise ValueError("gripper flange translation and quaternion must be configured together")
        return self

    @field_validator("robot_model")
    @classmethod
    def validate_robot_model(cls, value: str) -> str:
        if value.lower() != "fr3":
            raise ValueError("only the fr3 robot model is supported")
        return value.lower()

    @field_validator("robot_ip")
    @classmethod
    def validate_robot_ip(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("robot_ip must not be empty")
        return value.strip()

    @field_validator("gripper_flange_translation_m", mode="before")
    @classmethod
    def empty_translation_is_unset(cls, value: Any) -> Any:
        return None if value in (None, "", [], ()) else value

    @field_validator("gripper_flange_quaternion_xyzw", mode="before")
    @classmethod
    def empty_quaternion_is_unset(cls, value: Any) -> Any:
        return None if value in (None, "", [], ()) else value

    @property
    def origins(self) -> list[str]:
        return [origin.strip() for origin in self.allowed_origins.split(",") if origin.strip()]


@lru_cache(maxsize=1)
def get_settings() -> Settings:
    return Settings()
