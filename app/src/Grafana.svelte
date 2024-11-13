<script>
    // @ts-nocheck
    import { onMount } from "svelte";

    export let host;

    onMount(() => {
        const iframe = document.getElementById("grafana-iframe");
        iframe.src = `http://${host}:3001/d/jp2137/simba-dashboard?refresh=1s`;
    })

</script>

<iframe id="grafana-iframe"
    title="Grafana"
    frameborder="0">
</iframe>

<style>
    #grafana-iframe {
      position: fixed;
      top: 60px;
      left: 0;
      width: 100%;
      height: calc(100vh - 60px);
      border: none;
    }
</style>