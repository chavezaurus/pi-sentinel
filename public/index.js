var root = document.body

var count = 0 // added a variable

let Increment = function(e) {
    count++;
}

var Hello = {
    view: function() {
        return m("main", [
            m("h1", {class: "title"}, "Pi Sentinel"),
            // changed the next line
            m("button.pure-button", {onclick: Increment}, count + " clicks"),
        ])
    }
}

var tableItems = [
    {event: "s20190401_123456.mp4", keep: "Yes",   selected: false},
    {event: "s20190401_123556.mp4", keep: "Yes",   selected: false},
    {event: "s20190401_123656.mp4", keep: "Yes",   selected: false},
    {event: "s20190401_123756.mp4", keep: "Yes",   selected: false},
    {event: "s20190401_123856.mp4", keep: "Yes",   selected: false}
]

var selectedRow = null;

let ClickTable = function(e) {
    let row = e.target.parentElement.rowIndex-1;
    let column = e.target.cellIndex;

    if ( column === 1 ) {
        tableItems[row].keep = tableItems[row].keep === "Yes" ? "No" : "Yes";
    }

    if ( column === 0 ) {
        if ( selectedRow !== null ) {
            tableItems[selectedRow].selected = false;
        }

        selectedRow = row;
        tableItems[selectedRow].selected = true;
    }
}

var Table = {
    view: function() {
        return m("table.pure-table", [
            m("thead", m("tr", [
                m("th", "Event Videos"),
                m("th", "Keep")
            ])),
            m("tbody", {onclick: ClickTable}, tableItems.map( function(item,index){
                return [m(item.selected ? "tr.rselect" : "tr", [m("td", item.event), m("td", item.keep)])]
            }))
        ]
        )
    }
}

var Layout = {
    view: function() {
        return m("div.grid-container", [
            m("div.header", "Header"),
            m("div.sidebar", [m(Hello), m(Table)]),
            m("div.content", "Content"),
            m("div.footer", "Footer")
        ]);
    }
}

m.mount(root, Layout)
