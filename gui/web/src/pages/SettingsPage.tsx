import { Col, Row } from "antd";
import { SchemaSettingsComponent } from "../components/SchemaSettingsComponent.tsx";
import { useApi } from "../hooks/useApi.ts";
import { App } from "antd";

export const SettingsPage = () => {
    const guiApi = useApi();
    const { notification } = App.useApp();

    const findContainer = async (match: (c: any) => boolean, label: string) => {
        const res = await guiApi.containers.containersList();
        if (res.error) throw new Error(res.error.error);
        const container = res.data.containers?.find(match);
        if (!container?.id) throw new Error(`${label} container not found`);
        return container.id;
    };

    const restartMowgliNext = async () => {
        try {
            const id = await findContainer(
                (c) => c.names?.some((n: string) => n.includes("ros2")),
                "ROS2"
            );
            const res = await guiApi.containers.containersCreate(id, "restart");
            if (res.error) throw new Error(res.error.error);
            notification.success({ message: "ROS2 container restarted" });
        } catch (e: any) {
            notification.error({ message: "Failed to restart ROS2", description: e.message });
        }
    };

    const restartGui = async () => {
        try {
            const id = await findContainer(
                (c) => c.labels?.app === "gui" || c.names?.some((n: string) => n.includes("gui")),
                "GUI"
            );
            const res = await guiApi.containers.containersCreate(id, "restart");
            if (res.error) throw new Error(res.error.error);
            notification.success({ message: "GUI restarted" });
        } catch (e: any) {
            notification.error({ message: "Failed to restart GUI", description: e.message });
        }
    };

    return (
        <Row>
            <Col span={24}>
                <SchemaSettingsComponent
                    onRestartOM={restartMowgliNext}
                    onRestartGUI={restartGui}
                />
            </Col>
        </Row>
    );
};

export default SettingsPage;
