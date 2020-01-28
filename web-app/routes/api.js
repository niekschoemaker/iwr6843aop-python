var express = require('express');
var router = express.Router();
const { Connection, Request } = require("tedious");

var connected = true;
// Create connection to database
const config = {
    authentication: {
        options: {
            userName: "niekschoemaker",
            password: "password" // update me
        },
        type: "default"
    },
    server: "niekschoemaker.database.windows.net",
    options: {
        database: "iot",
        encrypt: true,
        rowCollectionOnRequestCompletion: true
    }
};

const connection = new Connection(config);

// Attempt to connect and execute queries if connection goes through
connection.on("connect", err => {
    if (err) {
        connected = false
    } else {
        connected = true;
    }
});

connection.on("end", () => {
    connection = null;
    connected = false;
})

function queryDatabase() {
    return new Promise((resolve, reject) => {
        var data = new Array();
        console.log("Reading rows from the Table...");
        const request = new Request(
            `SELECT TOP 1000 * FROM [iot].[dbo].[iwr6843]`,
            (err, rowCount, rows) => {
                if (err) {
                    console.error(err.message);
                } else {
                    console.log(`${rowCount} row(s) returned`);
                    resolve(data);
                }
            }
        );

        request.on("row", (columns) => {
            const columnDict = new Map();
            columns.forEach(column => {
                columnDict[column.metadata.colName] = column.value;
            });
            data.push(columnDict);
        })

        request.on("error", (err) => {
            reject(err);
        });

        connection.execSql(request);
    })
}

/* GET home page. */
router.get('/', async function (req, res, next) {
    if (!connected) {
        if(connection == null) {
            connection = new Connection(config);
            connection.on("connect", (err) => {
                if (err) {
                    console.error(err);
                } else {
                    connected = true;
                    res.json(await queryDatabase());
                    return;
                }
            });
        } else {
            res.status(500).send();
        }
    }
    res.json(await queryDatabase());
});

module.exports = router;
