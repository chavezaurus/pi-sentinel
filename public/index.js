var root = document.body;

var count = 0; // added a variable

var tableItems = [];

var selectedRow = null;

let Increment = function(e) {
    count++;
}

let UpdateEvents = function(fromDir) {
    m.request({
        method: "POST",
        url: fromDir,
        body: tableItems
    })
    .then(function(result) {
        result.sort();
        tableItems = result.map( function(item) {
            return {event: item, from: fromDir, to: fromDir, selected: false};
        });

        selectedRow = null;
    })
    .catch(function(e) {
        console.log( e.code );
    })
}

var DoTable = {
    view: function() {
        return m("pure-button-group", {role: "group", "aria-label": "Group"}, [
            m("button.pure-button", {onclick: function() {UpdateEvents("events")}}, "Events"),
            m("button.pure-button", {onclick: function() {UpdateEvents("saved")}},  "Saved"),
            m("button.pure-button", {onclick: function() {UpdateEvents("trash")}},  "Trash"),
        ])
    }
}

var videoSource = '';

let ClickTable = function(e) {
    let row = e.target.parentElement.rowIndex-1;
    let column = e.target.cellIndex;

    if ( column === 1 ) {
        var toDir = tableItems[row].to;

        if ( toDir === "events" ) {
            toDir = "trash";
        } else if ( toDir === "trash" ) {
            toDir = "saved";
        } else if ( toDir === "saved" ) {
            toDir = "events";
        }

        tableItems[row].to = toDir;
    }

    if ( column === 0 ) {
        if ( selectedRow !== null ) {
            tableItems[selectedRow].selected = false;
        }

        selectedRow = row;
        let rowObj = tableItems[row];
        rowObj.selected = true;
        videoSource = rowObj.from+'/'+rowObj.event;
    }
}

let Video = {
    view: function() {
        return m('video', {
            autoplay: true,
            style: 'sidth: 1920, margin: 0.5rem',
            controls: true,
            src: videoSource,
        })
    }
}

let Table = {
    view: function() {
        let toTable = {
            "events": "td.whiteCell",
            "trash":  "td.redCell",
            "saved":  "td.goldCell" };

        return m("table.pure-table", [
            m("thead", m("tr", [
                m("th", "Event Videos"),
                m("th", "Move-To")
            ])),
            m("tbody", {onclick: ClickTable}, tableItems.map( function(item,index){
                return [m(item.selected ? "tr.rselect" : "tr", 
                        [m("td", item.event), 
                        m(toTable[item.to], item.to)])];
            }))
        ]
        );
    }
}

let Layout = {
    view: function() {
        return m("div.grid-container", [
            m("h1.header", "Pi Sentinel"),
            m("div.controls", m(DoTable)),
            m("div.stable", m(Table)),
            m("div.content", m(Video)),
            m("div.footer", "Footer")
        ]);
    }
}

m.mount(root, Layout)
