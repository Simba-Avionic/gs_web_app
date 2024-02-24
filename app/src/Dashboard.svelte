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
        else if (path.id === 'valve1' || path.id === 'valve2')
        {
            path.style.stroke = "url(#gradient_red)";
        }

      });
    }

</script>


<div on:click={changeColor}>
    <SVG width=70vw />
</div>

<style>
</style>