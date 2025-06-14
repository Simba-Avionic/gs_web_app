<script>
    import { onMount, onDestroy, createEventDispatcher } from "svelte";
    import { rosTimeToFormattedTime, getStatusString } from "./Utils.svelte";

    import { slide } from "svelte/transition";
    let showTelemetry = false;

    function toggleTelemetry() {
        showTelemetry = !showTelemetry;
    }
    const dispatch = createEventDispatcher();

    const topicDataMap = {
        Recovery: [
            {
                topic: "mavlink/actuator",
                fields: [
                    {
                        name: "line_cutter",
                        extract: (data) =>
                            data.values & 0x08 ? "OPEN" : "CLOSED",
                        display: "Line Cutter",
                    },
                    {
                        name: "rec_valve",
                        extract: (data) =>
                            data.values & 0x10 ? "OPEN" : "CLOSED",
                        display: "REC Valve",
                    },
                ],
            },
        ],
        Avionics: [
            {
                topic: "mavlink/heartbeat",
                fields: [
                    {
                        name: "flight_computer_status",
                        extract: (data) =>
                            getStatusString(data.flight_computer_status),
                        display: "Status",
                    },
                ],
            },
        ],
        "Tank Pressure": [
            {
                topic: "mavlink/tank_pressure",
                fields: [
                    {
                        name: "pressure",
                        extract: (data) => data.pressure,
                        display: "Pressure",
                        unit: "bar",
                    },
                    {
                        name: "delta_pressure",
                        extract: (data) => data.d_pressure,
                        display: "Delta pressure",
                        unit: "bar",
                    },
                ],
            },
        ],
        "Tank Temperature": [
            {
                topic: "mavlink/tank_temperature",
                fields: [
                    {
                        name: "temperature1",
                        extract: (data) => data.temp1,
                        display: "Temperature1",
                        unit: "°C",
                    },
                    {
                        name: "temperature2",
                        extract: (data) => data.temp2,
                        display: "Temperature2",
                        unit: "°C",
                    },
                    {
                        name: "temperature3",
                        extract: (data) => data.temp3,
                        display: "Temperature3",
                        unit: "°C",
                    },
                ],
            },
        ],
        "Tank Actuators": [
            {
                topic: "mavlink/actuator",
                fields: [
                    {
                        name: "main_valve",
                        extract: (data) =>
                            data.values & 0x02 ? "OPEN" : "CLOSED",
                        display: "Main Valve",
                    },
                    {
                        name: "tank_vent",
                        extract: (data) =>
                            data.values & 0x04 ? "OPEN" : "CLOSED",
                        display: "Tank Vent",
                    },
                ],
            },
        ],
        Engine: [
            {
                topic: "mavlink/heartbeat",
                fields: [
                    {
                        name: "engine_computer_status",
                        extract: (data) =>
                            getStatusString(data.engine_computer_status),
                        display: "Engine Computer Status",
                    },
                ],
            },
            {
                topic: "mavlink/actuator",
                fields: [
                    {
                        name: "primer",
                        extract: (data) => (data.values & 0x01 ? "ON" : "OFF"),
                        display: "Igniter",
                    },
                ],
            },
        ],
    };

    export let host;
    export let title;

    let relevantTopicNames = [];
    let extractedData = {};
    let lastUpdated = null;
    let sockets = {};
    let isConnected = false;
    let hasNonOkStatus = false;

    function findRelevantTopicNames() {
        if (!topicDataMap[title]) return [];

        return topicDataMap[title]
            .map((topicConfig) => topicConfig.topic)
            .filter((value, index, self) => self.indexOf(value) === index); // Unique values
    }

    function processTopicData(topicName, data) {
        if (!topicDataMap[title]) return;

        let foundNonOkStatus = false;

        const topicConfigs = topicDataMap[title].filter(
            (config) => config.topic === topicName,
        );

        for (const config of topicConfigs) {
            for (const field of config.fields) {
                try {
                    const extractedValue = field.extract(data);
                    if (
                        field.name.includes("status") &&
                        extractedValue !== "OK" &&
                        typeof extractedValue === "string"
                    ) {
                        foundNonOkStatus = true;
                    }

                    if (!extractedData[field.name]) {
                        extractedData[field.name] = {};
                    }
                    extractedData[field.name] = {
                        value: extractedValue,
                        display: field.display || field.name,
                        unit: field.unit || "",
                    };
                } catch (e) {
                    console.error(
                        `Error extracting ${field.name} from ${topicName}:`,
                        e,
                    );
                }
            }
        }

        hasNonOkStatus = foundNonOkStatus;
        try {
            lastUpdated = {
                sec: data.header.stamp.sec,
                nanosec: data.header.stamp.nanosec,
            };
        } catch (e) {
            lastUpdated = null;
        }
    }

    function setupWebSockets() {
        relevantTopicNames.forEach((topicName) => {
            const socket = new WebSocket(`ws://${host}:8000/${topicName}`);

            socket.onopen = () => {
                console.log(`Connected to ${topicName}`);
                isConnected = true;
            };

            socket.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);

                    processTopicData(topicName, data);

                    dispatch("telemetryChange", {data, hasNonOkStatus});
                } catch (e) {
                    console.error(
                        `Error processing data from ${topicName}:`,
                        e,
                    );
                }
            };

            socket.onclose = () => {
                console.log(`Disconnected from ${topicName}`);
                if (Object.values(sockets).every((s) => s.readyState > 1)) {
                    isConnected = false;
                }
            };

            sockets[topicName] = socket;
        });
    }

    onMount(() => {
        relevantTopicNames = findRelevantTopicNames();
        setupWebSockets();
    });

    onDestroy(() => {
        Object.values(sockets).forEach((socket) => {
            if (socket && socket.readyState < 2) {
                socket.close();
            }
        });
    });
</script>

<div class="field">
    <!-- Top row: status, title area, button -->
    <div class="field-top-row">
        <div
            class="status-indicator {lastUpdated === 'None' &&
            lastUpdated === null &&
            lastUpdated === undefined
                ? 'red-status'
                : hasNonOkStatus
                  ? 'orange-status'
                  : 'green-status'}"
        ></div>

        <div class="field-header">
            <span class="field-text">{String(title)}</span>
            {#if lastUpdated != undefined && lastUpdated !== "None" && lastUpdated !== null}
                <span class="timestamp">
                    {rosTimeToFormattedTime(
                        lastUpdated.sec,
                        lastUpdated.nanosec,
                    )}
                </span>
            {/if}
        </div>

        <button on:click={toggleTelemetry} class="toggle-button">
            {showTelemetry ? "▲" : "▼"}
        </button>
    </div>

    <!-- Bottom row: telemetry data (full width) -->
    {#if extractedData != undefined && extractedData !== "None" && extractedData !== null && showTelemetry}
        <div
            in:slide={{ duration: 333 }}
            out:slide={{ duration: 333 }}
            class="telemetry-data"
        >
            <div class="fields-column">
                {#each Object.entries(extractedData) as [fieldName, fieldInfo]}
                    <div class="field-value">
                        <span class="field-label">{fieldName}:</span>
                        <span class="field-data">
                            {typeof fieldInfo.value === "number"
                                ? Number.isInteger(fieldInfo.value)
                                    ? fieldInfo.value
                                    : fieldInfo.value.toFixed(2)
                                : fieldInfo.value}
                            {fieldInfo.unit}
                        </span>
                    </div>
                {/each}
            </div>
        </div>
    {/if}
</div>

<style>
    .field {
        display: flex;
        flex-direction: column;
        width: 100%;
        text-align: left;
        box-sizing: border-box;
    }

    .field-top-row {
        display: flex;
        width: 100%;
        align-items: center;
    }

    .field-header {
        display: flex;
        flex-direction: row;
        align-items: center;
        flex-grow: 1;
        justify-content: space-between;
        margin-right: 0.75rem;
    }

    .field-text {
        font-weight: bold;
    }

    .timestamp {
        color: rgba(204, 204, 220, 0.65);
        font-size: 0.8em;
        margin-left: 8px;
    }

    .toggle-button {
        cursor: pointer;
        background: none;
        border: 1px solid rgba(204, 204, 220, 0.5);
        border-radius: 0.5rem;
        align-self: flex-start;
    }

    .telemetry-data {
        font-size: 0.95rem;
        padding-left: 0.5rem;
        width: 100%;
        box-sizing: border-box;
    }

    .fields-column {
        width: 100%;
    }

    .field-value {
        margin-top: 0.5rem;
        display: flex;
        justify-content: space-between;
        width: 100%;
    }

    .field-label {
        font-weight: bold;
        margin-right: 4px;
    }

    .field-data {
        text-align: right;
    }

    @media (min-width: 1920px) {
        .field {
            padding: 0.5rem;
            font-size: 1rem;
        }
        .telemetry-data {
            margin-top: 0.5rem;
            font-size: 1rem;
        }
        .field-top-row {
            font-size: 1.2rem;
        }
        .status-indicator {
            margin-right: 1.5rem;
        }
    }

    @media (max-width: 1280px) {
        .status-indicator {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            margin-right: 8px;
        }
    }
</style>
