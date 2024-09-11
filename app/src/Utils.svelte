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
  
  export function animatePath(paths, index) {
    paths[index].animate([{ opacity: "0" }, { opacity: "1" }], {
      duration: 1000,
      easing: "ease-out",
      fill: "forwards",
    }).onfinish = () => {
      index++;
      if (index >= paths.length) {
        index = 0;
      }
      animatePath(paths, index);
    };
  }
</script>