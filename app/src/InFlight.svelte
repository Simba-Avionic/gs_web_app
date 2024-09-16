<script>
    // @ts-nocheck
    import { onMount } from "svelte";
    import SVG from "../public/images/rocket.svg";
    import Field from "./Field.svelte";
</script>

<!-- svelte-ignore a11y-click-events-have-key-events -->
<!-- svelte-ignore a11y-no-static-element-interactions -->
<div id="svg-container">
    <SVG height="40vw" />
</div>


<style>

#svg-container {
    position: relative;
    width: 60vw;
}

</style>