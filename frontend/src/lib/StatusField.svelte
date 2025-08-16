<script>
  import { onMount, onDestroy } from "svelte";

  export let host;
  export let topic;
  export let field;
  export let title;

  let ws;
  let value = null;

  onMount(() => {
    ws = new WebSocket(`ws://${host}:8000/${topic}`);

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data[field] !== undefined) {
        value = data[field];
      }
    };
  });

  onDestroy(() => {
    if (ws) ws.close();
  });
</script>

<div class="status-field" class:no-data={value === null}>
  <div class="title">{title}</div>
  <div class="value">{value ?? "—"}</div>
</div>

<style>
.status-field {
  border: 1px solid var(--border-color);
  background: var(--snd-bg-color);
  border-radius: 0.5rem;
  padding: 0.5rem;
  text-align: center;
}
.status-field.no-data {
  opacity: 0.5;
}
.value {
  font-size: 1.4rem;
  font-weight: bold;
}
</style>
