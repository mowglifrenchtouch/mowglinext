import {useEffect, useState} from "react";
import {useWS} from "./useWS.ts";

export interface FusionOdom {
    header?: { stamp?: { sec: number; nanosec: number }; frame_id?: string };
    child_frame_id?: string;
    pose?: {
        pose?: {
            position?: { x: number; y: number; z: number };
            orientation?: { x: number; y: number; z: number; w: number };
        };
        covariance?: number[];
    };
    twist?: {
        twist?: {
            linear?: { x: number; y: number; z: number };
            angular?: { x: number; y: number; z: number };
        };
    };
}

/**
 * Subscribes to the raw /fusion/odom topic (nav_msgs/Odometry).
 * Unlike usePose which gets the dock-override AbsolutePose,
 * this provides the actual FusionCore UKF output.
 */
export const useFusionOdom = () => {
    const [odom, setOdom] = useState<FusionOdom>({})
    const stream = useWS<string>(() => {
            console.log({ message: "FusionOdom Stream closed" })
        }, () => {
            console.log({ message: "FusionOdom Stream connected" })
        },
        (e) => {
            setOdom(JSON.parse(e))
        })
    useEffect(() => {
        stream.start("/api/mowglinext/subscribe/fusionRaw")
        return () => { stream.stop() }
    }, []);
    return odom;
};
