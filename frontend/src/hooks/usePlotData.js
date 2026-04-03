import { queryFieldData } from '../services/api.js';
import { formatTimestamp } from '../utils/time.js';

export async function usePlotData(field, timeRange, timezone) {
    const res = await queryFieldData(field.topic, field.parent ? `${field.parent}/${field.val_name}` : field.val_name, timeRange);
    
    let rawX = res.records.map((entry) => new Date(entry._time).toISOString());
    let rawY = res.records.map((entry) => entry._value);

    if (field.type === "bool") rawY = rawY.map((v) => (v ? 1 : 0));

    const newestMs = rawX.length ? Date.parse(rawX[rawX.length - 1]) : Date.now();
    const windowStartMs = newestMs - timeRange * 60 * 1000;

    const paired = rawX.map((x, i) => ({
        tMs: Date.parse(x),
        tIso: x,
        v: rawY[i],
    }));
    
    const filtered = paired.filter((p) => p.tMs >= windowStartMs);
    const finalPairs = filtered.length ? filtered : paired.slice(-Math.max(1, Math.floor(timeRange * 60)));

    return {
        xData: finalPairs.map((p) => p.tIso),
        yData: finalPairs.map((p) => p.v),
        latestValue: rawY.length > 0 ? rawY[rawY.length - 1] : "--"
    };
}

export function formatPlotTicks(xData, timezone) {
    if (xData.length === 0) return { tickvals: [], ticktext: [] };

    const timestamps = xData.map((x) => Date.parse(x));
    const minTime = Math.min(...timestamps);
    const maxTime = Math.max(...timestamps);
    const range = maxTime - minTime;

    let tickCount = 6;
    const tickInterval = range / (tickCount - 1);

    const tickvals = [];
    const ticktext = [];

    for (let i = 0; i < tickCount; i++) {
        const tickTime = minTime + i * tickInterval;
        const tickDate = new Date(tickTime);
        const tickIso = tickDate.toISOString();

        tickvals.push(tickIso);
        ticktext.push(formatTimestamp(tickIso, timezone));
    }

    return { tickvals, ticktext };
}