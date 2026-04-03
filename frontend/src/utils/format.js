export function animatePath(paths, index) {
    paths[index].animate([{ opacity: "0" }, { opacity: "1" }], {
        duration: 1000,
        easing: "ease-out",
        fill: "forwards",
    }).onfinish = () => {
        index++;
        if (index >= paths.length) {
            index = 0;
        }
        animatePath(paths, index);
    };
}