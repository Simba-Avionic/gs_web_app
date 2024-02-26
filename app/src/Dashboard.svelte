<script>
    import SVG from "./assets/gs2.svg"

    function changeColor() {
      const svg = document.querySelector('svg');
      const paths = svg.querySelectorAll('path');
      paths.forEach(path => {
        console.log(path.id);

        if (path.id === 'oxidizer' || path.id === 'fuel')
        {
            path.style.fill = "url(#gradient_green)";
        }
        else if (path.id === 'valve1' || path.id === 'valve2' || path.id === 'hatch')
        {
            path.style.stroke = "url(#gradient_red)";
        }

      });
    }

</script>


<!-- svelte-ignore a11y-click-events-have-key-events -->
<!-- svelte-ignore a11y-no-static-element-interactions -->
<div on:click={changeColor} id="svg-container">
    <SVG width=70vw />
    <div id="div-elem">
        Some text!
    </div>
    <div id="div-elem2">
        Some text!
    </div>
</div>

<style>
    #svg-container {
        position: relative;
        width: 70vw;
    }

    #div-elem {
      position: absolute;
      top: 5%;
      left: -25%;
      width: 20%;
      height: 88%;
      border-radius: 1vw;
      border: 1px solid #eee;
      background-color: #242424;
    }

    #div-elem2 {
      position: absolute;
      top: 5%;
      left: 10%;
      width: 35%;
      height: 50%;
      border-radius: 1vw;
      border: 1px solid #eee;
      background-color: #242424;
    }

</style>