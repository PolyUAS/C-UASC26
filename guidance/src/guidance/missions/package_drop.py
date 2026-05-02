from enum import Enum

from guidance.missionFSM import AbstractFSMMetaclass


class PackageOpsMode(Enum):
    DROP = "drop"
    DELIVERY = "delivery"
    RECOVERY = "recovery"


class PackageOpsFSM(AbstractFSMMetaclass):
    def __init__(self, mode: PackageOpsMode):
        super().__init__(f"Package Ops ({mode.value})", time_limit=600.0)


class PackageDropFSM(PackageOpsFSM):
    def __init__(self):
        super().__init__(PackageOpsMode.DROP)


class PackageDeliveryFSM(PackageOpsFSM):
    def __init__(self):
        super().__init__(PackageOpsMode.DELIVERY)


class PackageRecoveryFSM(PackageOpsFSM):
    def __init__(self):
        super().__init__(PackageOpsMode.RECOVERY)
