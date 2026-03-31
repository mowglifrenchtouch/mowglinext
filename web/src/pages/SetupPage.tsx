import React, {useState} from 'react';
import {CheckCircleOutlined} from '@ant-design/icons'
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {Button, Card, Col, Row, Steps, Typography} from "antd";
import {FlashBoardComponent} from "../components/FlashBoardComponent.tsx";
import {SettingsComponent} from "../components/SettingsComponent.tsx";
import AsyncButton from "../components/AsyncButton.tsx";
import {FlashGPSComponent} from "../components/FlashGPSComponent.tsx";
import {SettingsConfig} from "../hooks/useSettings.ts";
import {useIsMobile} from "../hooks/useIsMobile";

const {Step} = Steps;

const SetupWizard: React.FC = () => {
    const {colors} = useThemeMode();
    const [currentStep, setCurrentStep] = useState(0);
    const isMobile = useIsMobile();

    const handleNext = () => {
        setCurrentStep(currentStep + 1);
    };

    const handlePrevious = () => {
        setCurrentStep(currentStep - 1);
    };

    const steps = [
        {
            title: 'Flash motherboard firmware',
            content: (
                <Card title={"Firmware configuration"} key={"flashBoard"}>
                    <FlashBoardComponent onNext={handleNext}/>
                </Card>
            ),
        },
        {
            title: 'Flash GPS configuration',
            content: (
                <Card title={"Flash GPS configuration"} key={"flashGPS"}>
                    <FlashGPSComponent onNext={handleNext} onPrevious={handlePrevious}/>
                </Card>
            ),
        },
        {
            title: 'Configure OpenMower',
            content: (
                <Card title={"Configure OpenMower"} key={"configureOpenMower"}>
                    <SettingsComponent contentStyle={{height: '55vh'}} actions={(form, save, restartOM, restartGUI) => {
                        return [
                            <Button onClick={handlePrevious}>Previous</Button>,
                            <Button type="primary" loading={form.loading} onClick={() => {
                                form.submit(async (values: SettingsConfig) => {
                                    await save(values);
                                    await restartOM();
                                    await restartGUI();
                                    handleNext();
                                }).catch(() => {});
                            }}>Save and restart</Button>,
                        ]
                    }}/>
                </Card>
            ),
        },
        {
            title: 'Setup complete',
            content: (
                <Card title={"Setup complete"} key={"complete"}>
                    <Row gutter={[16, 16]}>
                        <Col span={24} style={{textAlign: "center"}}>
                            <CheckCircleOutlined style={{fontSize: 48, color: colors.primary}}
                            />
                            <Typography.Title level={2}>Congratulations, your Mower is now fully
                                configured</Typography.Title>
                        </Col>
                        <Col span={24} style={{textAlign: "center"}}>
                            <AsyncButton onAsyncClick={async () => {
                                window.location.href = "/#/openmower";
                            }}>Go to dashboard</AsyncButton>
                        </Col>
                    </Row>
                </Card>
            )
        }
    ];


    return <Row gutter={[16, isMobile ? 16 : 32]}>
        <Col span={24}>
            <Typography.Text type="danger" style={{fontSize: isMobile ? 12 : 14}}>WARNING: This setup wizard will flash your
                motherboard firmware and the GPS configuration. Run at your own risk and be careful with voltage
                settings if you change them.</Typography.Text>
        </Col>
        <Col span={24}>
            <Steps current={currentStep} size={isMobile ? "small" : "default"} direction={isMobile ? "vertical" : "horizontal"}>
                {steps.map((step) => (
                    <Step key={step.title} title={step.title}/>
                ))}
            </Steps>
        </Col>
        <Col span={24}>
            <div className="steps-content">{steps[currentStep].content}</div>
        </Col>
    </Row>;
};

export default SetupWizard;