<script>
    // @ts-nocheck
    import { onMount } from "svelte";

    export let host;

    onMount(() => {
        const iframe = document.getElementById("grafana-iframe");
        iframe.src = `http://${host}:3001/dashboards`;
    })

</script>

<iframe id="grafana-iframe"
    title="Grafana"
    frameborder="0">
</iframe>

<style>
    #grafana-iframe {
      position: fixed;
      top: var(--navbar-height);
      left: 0;
      width: 100%;
      height: calc(100vh - var(--navbar-height));
      border: none;
    }
</style>