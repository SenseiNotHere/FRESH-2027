#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
from typing import Tuple

from wpilib import Timer, RobotController
from commands2 import Subsystem
from ntcore import NetworkTableInstance, StringPublisher, StringArrayPublisher
from wpimath.geometry import Rotation2d
from wpinet import PortForwarder
from pykit.logger import Logger

from utils import log


class LimelightCamera(Subsystem):
    def __init__(self, cameraName: str, isUsb0=False) -> None:
        super().__init__()

        self.cameraName = _fix_name(cameraName)

        instance = NetworkTableInstance.getDefault()
        self.table = instance.getTable(self.cameraName)
        self._path = self.table.getPath()

        self.pipelineIndexRequest = self.table.getDoubleTopic("pipeline").publish()
        self.pipelineIndex = self.table.getDoubleTopic("getpipe").getEntry(-1)
        # "cl" and "tl" are additional latencies in milliseconds

        self.ledMode = self.table.getIntegerTopic("ledMode").getEntry(-1)
        self.camMode = self.table.getIntegerTopic("camMode").getEntry(-1)
        self.tx = self.table.getDoubleTopic("tx").getEntry(0.0)
        self.ty = self.table.getDoubleTopic("ty").getEntry(0.0)
        self.ta = self.table.getDoubleTopic("ta").getEntry(0.0)
        self.hb = self.table.getIntegerTopic("hb").getEntry(0)

        self.lastHeartbeat = 0
        self.lastHeartbeatTime = 0
        self.heartbeating = False
        self.ticked = False

        self.takingSnapshotsWhenNoDetection = 0.0
        self.snapshotRequest = self.table.getIntegerTopic("snapshot").publish()
        self.snapshotRequestValue = self.table.getIntegerTopic("snapshot").getEntry(0).get()
        self.lastSnapshotRequestTime = 0.0

        # localizer state
        self.localizerSubscribed = False
        # if we want MegaTag2 localizer to work, we need to be publishing two things (to the camera):
        #   1. what robot's yaw is ("yaw=0 degrees" means "facing North", "yaw=90 degrees" means "facing West", etc.)
        #   2. where is this camera sitting on the robot (e.g. y=-0.2 meters to the right, x=0.1 meters fwd from center)
        self.robotOrientationSetRequest = self.table.getDoubleArrayTopic("robot_orientation_set").publish()
        self.cameraPoseSetRequest = self.table.getDoubleArrayTopic("camerapose_robotspace_set").publish()
        self.imuModeRequest = self.table.getIntegerTopic("imumode_set").publish()  # this is only for Limelight 4

        # port forwarding and feed address overrides, in case this camera is connected over USB
        self.isUsb0 = isUsb0
        self.ntSource: StringPublisher | None = None
        self.ntStreams: StringArrayPublisher | None = None
        self.ntStreamsValue: list[str] = []
        self.ntSourceValue = ""
        if isUsb0:
            self.setupCameraAtUsb0(instance)

    def setupCameraAtUsb0(self, instance: NetworkTableInstance):
        for port in [1180, 5800, 5801, 5802, 5803, 5804, 5805, 5806, 5807, 5808, 5809]:
            PortForwarder.getInstance().add(port, "172.29.0.1", port)
        teamNumber = RobotController.getTeamNumber()
        feedUrl = f"http://10.{teamNumber // 100}.{teamNumber % 100}.2:5800"
        publishedStreamInfo = instance.getTable("CameraPublisher").getSubTable(self.cameraName)
        self.ntStreams = publishedStreamInfo.getStringArrayTopic("streams").publish()
        self.ntStreamsValue = [f"mjpeg:{feedUrl}"]
        self.ntSource = publishedStreamInfo.getStringTopic("source").publish()
        self.ntSourceValue = f"ip:{feedUrl}"

    def addLocalizer(self):
        if self.localizerSubscribed:
            return

        self.localizerSubscribed = True
        # we can then receive the localizer results from the camera back
        self.botPose = self.table.getDoubleArrayTopic("botpose_orb_wpiblue").getEntry([])
        self.botPoseFlipped = self.table.getDoubleArrayTopic("botpose_orb_wpired").getEntry([])

    def setPipeline(self, index: int):
        self.pipelineIndexRequest.set(float(index))
        self.heartbeating = False  # wait until the next heartbeat before saying self.haveDetection == true

    def getPipeline(self) -> int:
        return int(self.pipelineIndex.get(-1))

    def getA(self) -> float:
        return self.ta.get()

    def getX(self) -> float:
        return self.tx.get()

    def getY(self) -> float:
        return self.ty.get()

    def getHB(self) -> float:
        return self.hb.get()

    def hasDetection(self):
        if self.getX() != 0.0 and self.heartbeating:
            return True

    def getSecondsSinceLastHeartbeat(self) -> float:
        return Timer.getFPGATimestamp() - self.lastHeartbeatTime

    def periodic(self) -> None:
        if self.ntStreams is not None and self.ntSource is not None:
            # keep overriding the feed info with the forwarded camera feed address
            self.ntStreams.set(self.ntStreamsValue)
            self.ntSource.set(self.ntSourceValue)

        now = Timer.getFPGATimestamp()
        heartbeat = self.getHB()
        self.ticked = False
        if heartbeat != self.lastHeartbeat:
            self.lastHeartbeat = heartbeat
            self.lastHeartbeatTime = now
            self.ticked = True
        heartbeating = now < self.lastHeartbeatTime + 5  # no heartbeat for 5s => stale camera
        if heartbeating != self.heartbeating:
            log("Vision", f"Camera {self.cameraName} is " + ("UPDATING" if heartbeating else "NO LONGER UPDATING"))
        self.heartbeating = heartbeating

        Logger.recordOutput(f"Vision/{self.cameraName}/Heartbeating", self.heartbeating)
        Logger.recordOutput(f"Vision/{self.cameraName}/HasDetection", bool(self.hasDetection()))
        Logger.recordOutput(f"Vision/{self.cameraName}/TX", self.getX())
        Logger.recordOutput(f"Vision/{self.cameraName}/TY", self.getY())
        Logger.recordOutput(f"Vision/{self.cameraName}/TA", self.getA())
        Logger.recordOutput(f"Vision/{self.cameraName}/Pipeline", self.getPipeline())
        Logger.recordOutput(f"Vision/{self.cameraName}/SecondsSinceHeartbeat", self.getSecondsSinceLastHeartbeat())

        if self.localizerSubscribed:
            tag_id = self.getAprilTagID()
            Logger.recordOutput(f"Vision/{self.cameraName}/AprilTagID", tag_id if tag_id is not None else -1)

        if heartbeating and self.takingSnapshotsWhenNoDetection and not self.hasDetection():
            if now > self.lastSnapshotRequestTime + self.takingSnapshotsWhenNoDetection:
                self.snapshotRequestValue += 1
                self.snapshotRequest.set(self.snapshotRequestValue)
                self.lastSnapshotRequestTime = now + self.takingSnapshotsWhenNoDetection

    def startTakingSnapshotsWhenNoDetection(self, secondsBetweenSnapshots=1.0):
        self.takingSnapshotsWhenNoDetection = secondsBetweenSnapshots

    def stopTakingSnapshotsWhenNoDetection(self):
        self.takingSnapshotsWhenNoDetection = 0.0

    def setPiPMode(self, mode: int):
        """
        Sets the picture-in-picture mode.
        :param mode: 0 = Side-by-Side, 1 = Secondary Camera in Lower-Right Corner
        """
        self.table.putNumber("stream", mode)

    def getAprilTagID(self) -> int | None:
        rawID = self.table.getString("tid", "")
        tagID = int(rawID) if rawID.isdigit() else None
        return tagID

    def getRedAprilTagID(self) -> int | None:
        rawID = self.table.getString("tid", "")
        tagID = int(rawID) if rawID.isdigit() else None
        return tagID


def _fix_name(name: str):
    if not name:
        name = "limelight"
    return name