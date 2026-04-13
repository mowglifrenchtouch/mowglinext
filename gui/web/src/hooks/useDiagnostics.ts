import {useEffect, useState} from "react";
import {useWS} from "./useWS.ts";

export interface DiagnosticStatus {
    level: number;
    name: string;
    message: string;
    hardware_id: string;
    values: { key: string; value: string }[];
}

export interface DiagnosticArray {
    status?: DiagnosticStatus[];
}

export const useDiagnostics = () => {
    const [diagnostics, setDiagnostics] = useState<DiagnosticArray>({})
    const diagnosticsStream = useWS<string>(() => {
            console.log({
                message: "Diagnostics Stream closed",
            })
        }, () => {
            console.log({
                message: "Diagnostics Stream connected",
            })
        },
        (e) => {
            setDiagnostics(JSON.parse(e))
        })
    useEffect(() => {
        diagnosticsStream.start("/api/mowglinext/subscribe/diagnostics")
        return () => {
            diagnosticsStream.stop()
        }
    }, []);
    return {diagnostics, stop: diagnosticsStream.stop, start: diagnosticsStream.start};
};
