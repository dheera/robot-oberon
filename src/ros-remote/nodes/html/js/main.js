var theIo = null;

$(function() {
  console.log("initializing");
  theIo = io('/robot');

  theIo.on("connect", function(msg) {
    console.log("connected");
  });

  theIo.on("reconnect", function(msg) {
    console.log("reconnected");
  });

  theIo.on("rosout", function(msg) {
    console.log(msg);
  });

  theIo.on("status", function(msg) {
    console.log(msg);
  });

  theIo.on("disconnect", function(msg) {
    console.log("disconnected");
  });
});
