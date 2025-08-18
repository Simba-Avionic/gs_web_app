<script>
  import { onMount, onDestroy } from "svelte";

  export let host;
  export let item;

  let ws;
  let value = null;

  onMount(() => {
    ws = new WebSocket(`ws://${host}/${item.topic}`);

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data[item.field] !== undefined) {
        value = data[item.field];
      }
    };
  });

  onDestroy(() => {
    if (ws) ws.close();
  });
</script>

<div class="status-field" class:no-data={value === null || value === "None"}>
  <div class="title">{item.title}</div>
  <div class="value">{value ?? "—"}</div>
</div>

<style>
  .status-field {
    border: 1px solid var(--border-color);
    background: var(--snd-bg-color);
    border-radius: 0.5rem;
    padding: 0.5rem;
    text-align: center;
    display: flex;
    flex-direction: column;
    justify-content: center;
  }

  .title {
    font-size: 0.9em;
    text-align: center;
    /* padding: 0.4rem; */

    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
    position: relative;
  }

  .status-field.no-data {
    opacity: 0.6;
  }
  .status-field {
    transition: opacity 0.5s ease-in-out; /* smooth opacity changes */
  }
  .value {
    font-size: 1.3rem;
    font-weight: bold;
  }
</style>
