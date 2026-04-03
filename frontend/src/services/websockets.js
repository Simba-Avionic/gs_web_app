import { writable } from "svelte/store";

const wsConnections = {};
const subscribers = {}; // topic_name → array of callbacks

function getWebSocket(host, topic) {
    if (wsConnections[topic]) return wsConnections[topic];

    const ws = new WebSocket(`ws://${host}/${topic}`);
    wsConnections[topic] = ws;

    ws.onopen = () => {
        console.log(`Connected to ${topic}`);
        // console.log("Total WebSocket connections:", Object.keys(wsConnections).length);
    }
    ws.onclose = () => {
        console.warn(`Connection closed: ${topic}`);
        delete wsConnections[topic];
        // console.log("Total WebSocket connections:", Object.keys(wsConnections).length);
    };
    ws.onerror = (err) => {
        console.error(`WebSocket error on ${topic}:`, err);
    }

    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            (subscribers[topic] || []).forEach((cb) => cb(data));
        } catch (err) {
            console.error(`Error parsing data for ${topic}:`, err);
        }
    };

    return ws;
}

export function subscribeToTopic(host, topic, callback) {
    if (!subscribers[topic]) {
        subscribers[topic] = [];
        getWebSocket(host, topic); // ensure connection is opened
    }
    subscribers[topic].push(callback);

    // console.log(`Subscribed to ${topic}. Subscribers for this topic: ${subscribers[topic].length}`);
    // console.log("Total subscribers (all topics):",
    //     Object.values(subscribers).reduce((acc, arr) => acc + arr.length, 0)
    // );

    return () => {
        subscribers[topic] = subscribers[topic].filter((cb) => cb !== callback);
        if (subscribers[topic].length === 0) {
            wsConnections[topic]?.close();
            delete wsConnections[topic];
            delete subscribers[topic];   // <--- add this line
        }

        // console.log(`Unsubscribed from ${topic}. Subscribers left for this topic: ${subscribers[topic]?.length || 0}`);
        // console.log("Total subscribers (all topics):",
        //     Object.values(subscribers).reduce((acc, arr) => acc + arr.length, 0)
        // );
    };
}
