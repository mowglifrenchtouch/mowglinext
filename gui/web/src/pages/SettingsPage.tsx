import { Col, Row } from "antd";
import { SchemaSettingsComponent } from "../components/SchemaSettingsComponent.tsx";
import { useApi } from "../hooks/useApi.ts";
import { App } from "antd";

export const SettingsPage = () => {
    const guiApi = useApi();
    const { notification } = App.useApp();

    const restartMowgliNext = async () => {
        try {
            const resContainersList = await guiApi.containers.containersList();
            if (resContainersList.error) throw new Error(resContainersList.error.error);
            const container = resContainersList.data.containers?.find(
                (c) => c.labels?.app === "mowglinext" || c.names?.includes("/mowglinext")
            );
            if (container?.id) {
                const res = await guiApi.containers.containersCreate(container.id, "restart");
                if (res.error) throw new Error(res.error.error);
                notification.success({ message: "MowgliNext restarted" });
            } else {
                throw new Error("MowgliNext container not found");
            }
        } catch (e: any) {
            notification.error({ message: "Failed to restart MowgliNext", description: e.message });
        }
    };

    const restartGui = async () => {
        try {
            const resContainersList = await guiApi.containers.containersList();
            if (resContainersList.error) throw new Error(resContainersList.error.error);
            const container = resContainersList.data.containers?.find(
                (c) => c.labels?.app === "gui" || c.names?.includes("/mowglinext")
            );
            if (container?.id) {
                const res = await guiApi.containers.containersCreate(container.id, "restart");
                if (res.error) throw new Error(res.error.error);
                notification.success({ message: "GUI restarted" });
            } else {
                throw new Error("GUI container not found");
            }
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
