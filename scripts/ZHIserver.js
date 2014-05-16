var net = require('net');
var fs = require('fs');
var http = require('http');
var path = require('path');
var express = require("express");

var app = express();

try
{
  deleteFolderRecursive(path.join("tmp", "unwarp"));
}
catch(ex){

}

//create files
var exist = fs.existsSync("tmp");
if(!exist){
  fs.mkdirSync("tmp");
}
else{
  exist = fs.existsSync(path.join("tmp", "unwarp"));
  if(!exist){
    fs.mkdirSync(path.join("tmp", "unwarp"));
  }
}

var RATE = 90;


app.get("/" , function(req, res){
    //localhost:9001/?rate=90
    RATE = req.query.rate;
    console.log(RATE);
    res.end("end")
})



var i = 0;

function saveJPG(){
    var server = http.get("http://192.168.1.144:8080/shot.jpg", function(res){
        //console.log("make request");
        var data = new Buffer(0);
        res.on('data', function(chunk){
            data = Buffer.concat([data, chunk]);
        });

        res.on('error',function(e){console.error(e)})

        res.on('end', function(){
            fs.writeFile(path.join("tmp", "unwarp" , "image" + NumString(i) + ".jpeg"), data);

            if(i%100 == 0)
            {
                console.log("printed: " + i);
            }
            if(i > 30)
            {
                try
                {
                  fs.unlink(path.join("tmp", "unwarp" , "image" + NumString(i - 30 - 1) + ".jpeg"),function(){});
                }
                catch(ex)
                {

                }
            }
            i++;
        })
    });    

    server.on('error',function(e){console.error(e)});

    setTimeout(saveJPG, RATE);
}

function NumString(i){
  if(i < 10){
    return "000000000" + i.toString();
  }
  if(i < 100){
    return "00000000" + i.toString();
  }
  if(i < 1000){
    return "0000000" + i.toString();
  }
  if(i < 10000){
    return "000000" + i.toString();
  }
  if(i < 100000){
    return "00000" + i.toString();
  }
  if(i < 1000000){
    return "0000" + i.toString();
  }
  if(i < 10000000){
    return "000" + i.toString();
  }
  if(i < 100000000){
    return "00" + i.toString();
  }
  if(i < 1000000000){
    return "0" + i.toString();
  }
  return i.toString();
}

function deleteFolderRecursive(path){
  if( fs.existsSync(path) ) {
    fs.readdirSync(path).forEach(function(file,index){
      var curPath = path + "/" + file;
      if(fs.lstatSync(curPath).isDirectory()) { // recurse
        deleteFolderRecursive(curPath);
      } else { // delete file
        fs.unlinkSync(curPath);
      }
    });
    fs.rmdirSync(path);
  }
};

saveJPG();
app.listen(9001);


