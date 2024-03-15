<script>
  let latestNumber = null;

  import { writable } from "svelte/store";

  // const messages = writable([]);
  const evtSource = new EventSource("http://localhost:8000/number");
  evtSource.onmessage = function (event) {
    // console.log(event);
    // var dataobj = JSON.parse(event.data);
    latestNumber = event.data;
    // messages.update((arr) => arr.concat(dataobj));
  };
</script>

<div>
  {#if latestNumber !== null}
    <h3>Latest Number: {latestNumber}</h3>
  {:else}
    <h3>Loading...</h3>
  {/if}
</div>
