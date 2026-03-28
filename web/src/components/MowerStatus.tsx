import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {App, Badge, Dropdown, Modal, Space, Typography} from "antd";
import {PoweroffOutlined, ReloadOutlined, DesktopOutlined, WifiOutlined} from "@ant-design/icons"
import {stateRenderer} from "./utils.tsx";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {useApi} from "../hooks/useApi.ts";
import type {MenuProps} from "antd";

const pulseKeyframes = `
@keyframes mowerPulseGreen {
    0%, 100% { box-shadow: 0 0 0 0 rgba(82, 196, 26, 0.6); }
    50% { box-shadow: 0 0 0 4px rgba(82, 196, 26, 0); }
}
@keyframes mowerPulseRed {
    0%, 100% { box-shadow: 0 0 0 0 rgba(255, 77, 79, 0.6); }
    50% { box-shadow: 0 0 0 4px rgba(255, 77, 79, 0); }
}
`;

const statusColor = (state: string | undefined, colors: {primary: string; warning: string; danger: string}): string => {
    switch (state) {
        case "MOWING":
        case "DOCKING":
        case "UNDOCKING":
            return colors.primary;
        case "IDLE":
            return colors.warning;
        default:
            return colors.danger;
    }
};

export const MowerStatus = () => {
    const {colors} = useThemeMode();
    const {highLevelStatus} = useHighLevelStatus();
    const guiApi = useApi();
    const {notification} = App.useApp();
    const gpsPercent = Math.round((highLevelStatus.GpsQualityPercent ?? 0) * 100);
    const batteryPercent = Math.round((highLevelStatus.BatteryPercent ?? 0) * 100);

    const isMowing = highLevelStatus.StateName === "MOWING" || highLevelStatus.StateName === "DOCKING" || highLevelStatus.StateName === "UNDOCKING";
    const isEmergency = !!highLevelStatus.Emergency;
    const pulseAnimation = isEmergency
        ? 'mowerPulseRed 1.5s ease-in-out infinite'
        : isMowing
            ? 'mowerPulseGreen 2s ease-in-out infinite'
            : 'none';

    const hasArea = highLevelStatus.CurrentArea !== undefined && highLevelStatus.CurrentArea >= 0;
    const hasProgress = isMowing && highLevelStatus.CurrentPathIndex !== undefined && highLevelStatus.CurrentPath !== undefined && highLevelStatus.CurrentPath > 0;
    const progressPercent = hasProgress
        ? Math.round(((highLevelStatus.CurrentPathIndex ?? 0) / (highLevelStatus.CurrentPath ?? 1)) * 100)
        : null;

    const restartMowgli = async () => {
        try {
            const res = await guiApi.containers.containersList();
            if (res.error) throw new Error(res.error.error);
            const container = res.data.containers?.find(
                (c) => c.labels?.app === "openmower" || c.names?.includes("/openmower")
            );
            if (container?.id) {
                const cmdRes = await guiApi.containers.containersCreate(container.id, "restart");
                if (cmdRes.error) throw new Error(cmdRes.error.error);
                notification.success({message: "Mowgli restarted"});
            } else {
                throw new Error("OpenMower container not found");
            }
        } catch (e: any) {
            notification.error({message: "Failed to restart Mowgli", description: e.message});
        }
    };

    const rebootSystem = async () => {
        try {
            await guiApi.request({path: "/system/reboot", method: "POST"});
            notification.success({message: "Rebooting..."});
        } catch (e: any) {
            notification.error({message: "Failed to reboot", description: e.message});
        }
    };

    const shutdownSystem = async () => {
        try {
            await guiApi.request({path: "/system/shutdown", method: "POST"});
            notification.success({message: "Shutting down..."});
        } catch (e: any) {
            notification.error({message: "Failed to shutdown", description: e.message});
        }
    };

    const confirmAction = (title: string, content: string, onOk: () => Promise<void>) => {
        Modal.confirm({
            title,
            content,
            okText: "Confirm",
            okType: "danger",
            cancelText: "Cancel",
            onOk,
        });
    };

    const powerMenuItems: MenuProps["items"] = [
        {
            key: "restart-mowgli",
            icon: <ReloadOutlined/>,
            label: "Restart Mowgli",
            onClick: () => confirmAction("Restart Mowgli", "This will restart the OpenMower container.", restartMowgli),
        },
        {type: "divider"},
        {
            key: "reboot",
            icon: <DesktopOutlined/>,
            label: "Reboot Raspberry Pi",
            onClick: () => confirmAction("Reboot Raspberry Pi", "The system will reboot. You will lose connection temporarily.", rebootSystem),
        },
        {
            key: "shutdown",
            icon: <PoweroffOutlined/>,
            label: "Shutdown Raspberry Pi",
            danger: true,
            onClick: () => confirmAction("Shutdown Raspberry Pi", "The system will shut down. You will need physical access to turn it back on.", shutdownSystem),
        },
    ];

    return (
        <>
            <style>{pulseKeyframes}</style>
            <Space size="small" style={{flexShrink: 0}}>
                <Space size={4}>
                    <Badge
                        color={statusColor(highLevelStatus.StateName, colors)}
                        style={{animation: pulseAnimation, borderRadius: '50%'}}
                    />
                    <Typography.Text style={{fontSize: 12, color: colors.text, whiteSpace: 'nowrap'}}>
                        {stateRenderer(highLevelStatus.StateName)}
                    </Typography.Text>
                </Space>
                {isMowing && hasArea && (
                    <Typography.Text style={{fontSize: 11, color: colors.primary, whiteSpace: 'nowrap'}}>
                        A{(highLevelStatus.CurrentArea ?? 0) + 1}
                        {progressPercent !== null ? ` ${progressPercent}%` : ''}
                    </Typography.Text>
                )}
                <Space size={4}>
                    <WifiOutlined style={{color: gpsPercent > 0 ? colors.primary : colors.danger, fontSize: 13}}/>
                    <Typography.Text style={{fontSize: 12, color: colors.text}}>
                        {gpsPercent}%
                    </Typography.Text>
                </Space>
                <Dropdown menu={{items: powerMenuItems}} trigger={["click"]} placement="bottomRight">
                    <Space size={4} style={{cursor: "pointer"}}>
                        <PoweroffOutlined style={{
                            color: highLevelStatus.IsCharging ? colors.primary : colors.muted,
                            fontSize: 13,
                        }}/>
                        <Typography.Text style={{fontSize: 12, color: colors.text}}>
                            {batteryPercent}%
                        </Typography.Text>
                    </Space>
                </Dropdown>
            </Space>
        </>
    );
}
