<script>
    import mysvg from "./assets/gs2.svg"
    import GS from "./GS.svelte"

    import SVG from "./assets/gs.svg"

    function changeColor() {
      // You can manipulate the SVG elements dynamically using JavaScript
      const svg = document.querySelector('.my-svg svg');
      const paths = svg.querySelectorAll('path'); // Select all path elements
      paths.forEach(path => {
        path.style.fill = 'red'; // Change fill color to red on click
      });
    }
</script>

<div>
    <!-- <img src={mysvg} alt="gs">  -->
    <SVG width=1000 />
    <!-- {@html GS} -->
</div>

<style>
    img {
        width: 50vw;
    }
    .svg {
        width: 50vw;
    }

</style>