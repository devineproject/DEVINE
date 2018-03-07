import $ from 'cash-dom';

export default function route(command, replace=false) {
    if(command[0] === "/")
        command = command.slice(1);
    if(replace)
        history.replaceState({}, "", command);
    else
        history.pushState({}, "", command);

    const currentView = $(`.command-view.command-current`);
    const nextView = $(`.command-view[name="${command}"]`);
    if(currentView.length > 0 && nextView.length > 0) {
        currentView[0].setAttribute('hidden', true);
        nextView[0].removeAttribute('hidden');
        currentView[0].classList.remove('command-current');
        currentView[0].classList.add('command-current');
    } else {
        console.log('No such command!');
    }
}

$('.command-search')[0].focus();
$('body').on('keydown', (e) => {
    if(e.keyCode === 191) // '/' to search
        $('.command-search')[0].focus();
});

const views = $('.command-view');
views.each((view) => {
    const link = $(`.command-menu.command-${view.getAttribute('name')}`);
    if(link.length > 0)
        link.eq(0).on("click", "li", () => {
            route(view.getAttribute('name'));
        });
});
