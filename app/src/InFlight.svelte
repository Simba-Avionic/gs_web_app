<script>
    // @ts-nocheck
    import { onMount } from "svelte";
    import SVG from "../public/images/rocket_inflight.svg";
    import Field from "./Field.svelte";

    export let host;

</script>

<!-- svelte-ignore a11y-click-events-have-key-events -->
<!-- svelte-ignore a11y-no-static-element-interactions -->
<div class='in-flight'>
    <div id="svg-container">
        <SVG height="40vw" />
    </div>
    
    <div class="rocket-field" id="recovery">
        <p>Recovery</p>
    </div>
    
    <div class="rocket-field" id="payload">
        <p>Payload</p>
    </div>
    
    <div class="rocket-field" id="avionics">
        <p>Avionics</p>
    </div>
    
    <div class="rocket-field" id="tank">
        <p>Tank</p>
    </div>
    
    <div class="rocket-field" id="engine">
        <p>Engine</p>
    </div>
</div>


<style>

.rocket-field {
    border: 1px solid rgba(204, 204, 220, 0.5);
    border-radius: 1vw;
    width: 12vw;
    height: 8vh;
}

#svg-container {
    position: absolute;
    top: 20vh;
    left: 15vw;
}

#recovery {
    position: absolute;
    top: 24vh;
    left: 46vw;
}

#payload {
    position: absolute;
    top: 30vh;
    left: 5vw;
}

#avionics {
    position: absolute;
    top: 50vh;
    left: 46vw;
}

#tank {
    position: absolute;
    top: 68vh;
    left: 5vw;
}

#engine {
    position: absolute;
    top: 70vh;
    left: 46vw;
}

</style>