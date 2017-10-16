function (event) {

    function handle_event (symbol, value) {
        var output = value.toFixed(2) + " BPM";
        switch (symbol) {
            case 'CP_TEMPO_OUT':
                event.icon.find ('[mod-port-symbol=CP_TEMPO_OUT]').text(output);
                break;
            default:
                break;
        }
    }

    if (event.type == 'start') {
        var ports = event.ports;
        for (var p in ports) {
            handle_event (ports[p].symbol, ports[p].value);
        }
    }
    else if (event.type == 'change') {
        handle_event (event.symbol, event.value);
    }
}
