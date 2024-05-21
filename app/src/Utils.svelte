<script context="module">
    export function rosTimeToFormattedTime(secs, nsecs) {
        const milliseconds = secs * 1000 + nsecs / 1000000;
        const date = new Date(milliseconds);

        const options = {
        year: "numeric",
        month: "2-digit",
        day: "2-digit",
        hour: "2-digit",
        minute: "2-digit",
        second: "2-digit",
        timeZone: "UTC",
        };

        // @ts-ignore
        return date.toLocaleString("en-GB", options);
  }
</script>