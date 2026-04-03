export function rosTimeToFormattedTime(iso = false, secs, nsecs) {
    if (typeof secs !== "number" || typeof nsecs !== "number") {
        console.error("Invalid ROS time:", secs, nsecs);
        return "--";
    }
    const ms = secs * 1000 + nsecs / 1e6;
    if (isNaN(ms) || ms <= 0) {
        console.error("Invalid milliseconds:", ms);
        return "--";
    }

    const date = new Date(ms);

    if (iso) {
        return new Date(ms).toISOString();
    }

    const options = {
        year: "numeric",
        month: "2-digit",
        day: "2-digit",
        hour: "2-digit",
        minute: "2-digit",
        second: "2-digit",
        timeZone: import.meta.env.VITE_TIMEZONE,
    };

    // @ts-ignore
    return date.toLocaleString("en-GB", options);
}

export function formatTimestamp(isoString, timezone) {
    const date = new Date(isoString);
    const options = {
        hour: "2-digit",
        minute: "2-digit",
        second: "2-digit",
        timeZone: timezone,
    };
    // @ts-ignore
    return date.toLocaleString("en-GB", options);
}