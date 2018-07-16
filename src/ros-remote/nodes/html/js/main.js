var theIo = null;
var touchStartX, touchStartY, touchCurrentX, touchCurrentY;
var touchInterval;

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

  $('.control').bind('touchstart', function(e) {
    touchStartX = e.originalEvent.touches[0].clientX;
    touchStartY = e.originalEvent.touches[0].clientY;
    touchCurrentX = touchStartX;
    touchCurrentY = touchStartY;
    touchInterval = setInterval(touching, 100);
  });

  $('.control').bind('touchmove',function(e) {
    e.preventDefault();
    touchLastTime = Date.now();
    touchCurrentX = e.originalEvent.touches[0].clientX;
    touchCurrentY = e.originalEvent.touches[0].clientY;
  });

  $('.control').bind('touchend', function(e) {
    touchCurrentX = 0;
    touchCurrentY = 0;
    clearInterval(touchInterval);
  });
  $('.control').bind('touchleave', function(e) {
    touchCurrentX = 0;
    touchCurrentY = 0;
    clearInterval(touchInterval);
  });
  $('.control').bind('touchcancel', function(e) {
    touchCurrentX = 0;
    touchCurrentY = 0;
    clearInterval(touchInterval);
  });

});

function touching() {
  $('.messages').text('touching');
  var translationX = Math.floor(touchCurrentX - touchStartX) / $('.control')[0].clientWidth * 2;
  var translationY = Math.floor(touchCurrentY - touchStartY) / $('.control')[0].clientHeight * 2;
  if(translationX > 1) translationX = 1;
  if(translationX < -1) translationX = -1;
  if(translationY > 1) translationY = 1;
  if(translationY < -1) translationY = -1;
  $('.messages').text(translationX + "," +  translationY);
  theIo.emit("motion", [-translationY, translationX, 0.0]);
}
