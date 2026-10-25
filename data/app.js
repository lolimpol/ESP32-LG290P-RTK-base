(function(){
'use strict';
var $=function(s){return document.querySelector(s)},tab='dash',names={};
function esc(s){return String(s).replace(/[&<>"]/g,function(c){return{'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]})}
function api(p,o){return fetch(p,o).then(function(r){return r.text().then(function(t){var j;try{j=JSON.parse(t)}catch(e){j={ok:r.ok,error:t}}j._s=r.status;return j})})}

/* ---------- formatting ---------- */
function dur(s){var d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60);return(d?d+'d ':'')+h+'h '+m+'m '+(s%60)+'s'}
function num(v){return Number(v).toLocaleString()}
function fmt(k,v){
  if(typeof v==='boolean')return'<span class="'+(v?'y':'n')+'">'+(v?'yes':'no')+'</span>';
  if(typeof v==='number'){
    if(/ageMs$/i.test(k))return v<0?'–':(v<1000?v+' ms':(v/1000).toFixed(1)+' s')+' ago';
    if(k==='uptime')return dur(v);
    if(/Unix$/.test(k))return v?new Date(v*1000).toLocaleString():'never';
    if(/^(lat|lon)$/.test(k))return v.toFixed(8);
    if(/^(heap|heapMin|heapBlock|stackFree|rxBuffer)$/.test(k))return num(v)+' B';
    if(/(Ms|heartbeatMs)$/.test(k))return num(v)+' ms';
    if(/(Sec|S)$/.test(k)&&k!=='ageMs')return num(v)+' s';
    if(/bps$|bytesPerSec$/i.test(k))return num(v)+' B/s';
    return Number.isInteger(v)?num(v):v;
  }
  var c=/error|SILENT|unsynchronised/i.test(v)&&v!==''?' class="e"':'';
  return'<span'+c+'>'+(v===''?'–':esc(v))+'</span>';
}
function render(v,k){
  if(Array.isArray(v)){
    if(!v.length)return'<span class="n">–</span>';
    if(typeof v[0]!=='object')return v.map(function(x){return'<div>'+esc(x)+'</div>'}).join('');
    var cols=Object.keys(v[0]);
    return'<table><tr>'+cols.map(function(c){return'<th>'+esc(c)+'</th>'}).join('')+'</tr>'+
      v.map(function(r){return'<tr>'+cols.map(function(c){return'<td>'+fmt(c,r[c])+'</td>'}).join('')+'</tr>'}).join('')+'</table>';
  }
  if(v&&typeof v==='object'){
    var kv='',sub='';
    Object.keys(v).forEach(function(x){
      if(v[x]&&typeof v[x]==='object')sub+='<h3>'+esc(x)+'</h3>'+render(v[x],x);
      else kv+='<dt>'+esc(x)+'</dt><dd>'+fmt(x,v[x])+'</dd>';
    });
    return(kv?'<dl>'+kv+'</dl>':'')+sub;
  }
  return fmt(k,v);
}
var TITLES={sys:'System',net:'Wi-Fi',gnss:'GNSS / LG290P',rtcm:'RTCM',ntrip:'NTRIP uploads',caster:'NTRIP caster',ntp:'NTP server',log:'USB / logging'};
var WIDE={rtcm:1,ntrip:1};
function drawStatus(s){
  $('#fwv').textContent='v'+s.sys.fw;
  s.rtcm.messages.forEach(function(m){names[m.msg]=m.name});
  var h=s.warn.length?'<div class="card warnbox"><h2>Attention</h2>'+s.warn.map(function(w){return'<div>'+esc(w)+'</div>'}).join('')+'</div>':'';
  Object.keys(TITLES).forEach(function(k){
    h+='<div class="card'+(WIDE[k]?' wide':'')+'"><h2>'+TITLES[k]+'</h2>'+render(s[k],k)+'</div>';
  });
  $('#dash').innerHTML=h;
}
var busy=false;
function poll(){
  if((tab!=='dash'&&tab!=='rx')||document.hidden||busy)return;
  busy=true;
  var ac=new AbortController(),to=setTimeout(function(){ac.abort()},6000);
  api('/api/status',{cache:'no-store',signal:ac.signal}).then(function(s){
    if(s.sys){if(tab==='dash')drawStatus(s);else drawRx(s);$('#dot').className='dot '+(s.gnss.uart==='receiving'?'ok':'bad');$('#upd').textContent='updated '+new Date().toLocaleTimeString()}
    else{$('#dot').className='dot bad';$('#upd').textContent='bad response ('+s._s+')'}
  }).catch(function(){$('#dot').className='dot bad';$('#upd').textContent='no response from device'})
  .then(function(){clearTimeout(to);busy=false});
}

/* ---------- receiver command tab ---------- */
var QUICK=[['Enable RMC','PQTMCFGMSGRATE,W,RMC,1'],['Enable GGA','PQTMCFGMSGRATE,W,GGA,1'],['Read RMC rate','PQTMCFGMSGRATE,R,RMC'],['Firmware version','PQTMVERNO'],['Save to receiver','PQTMSAVEPAR']];
$('#quick').innerHTML=QUICK.map(function(q,i){return'<button data-q="'+i+'">'+esc(q[0])+'</button>'}).join('');
$('#quick').addEventListener('click',function(e){var i=e.target.getAttribute('data-q');if(i!==null)$('#cmd').value=QUICK[i][1]});
function drawRx(s){
  var g=s.gnss;
  $('#rxinfo').innerHTML=(g.txPin<0?'<span class="e">GNSS TX pin is disabled (-1): commands cannot be sent.</span>':'ESP32 TX GPIO '+g.txPin+' → LG290P RX')+' · commands sent '+g.cmdSent+' · replies '+g.cmdReplies;
  $('#replies').innerHTML=g.replies.length?g.replies.map(function(r){return'<div>'+esc(r)+'</div>'}).join(''):'<span class="n">none yet</span>';
}
$('#bSend').onclick=function(){
  var c=$('#cmd').value.trim();if(!c)return;msg($('#cmdmsg'),'Sending…',1);
  fetch('/api/gnsscmd',{method:'POST',headers:{'Content-Type':'text/plain'},body:c}).then(function(r){return r.json()}).then(function(r){
    msg($('#cmdmsg'),r.ok?'Sent: '+r.sent:'Rejected: '+r.error,r.ok);setTimeout(poll,600);setTimeout(poll,1800);
  }).catch(function(){msg($('#cmdmsg'),'Request failed.',0)});
};

/* ---------- settings form schema: [key,label,type,hint] ---------- */
var YN=[['0','no'],['1','yes']];
var SCHEMA=[
['Web login & network',[['web_user','Web user','t'],['web_pass','Web password','p'],['hostname','Hostname','t'],
  ['wifi_ssid','Wi-Fi SSID','t'],['wifi_pass','Wi-Fi password','p','Empty only for open networks; otherwise 8-63 characters.']]],
['Base station position (RTCM 1005)',[['base_x','ECEF X (m)','n','Cartesian metres of the antenna reference point (ARP), 0.1 mm resolution, in the frame/epoch your rovers expect (e.g. ITRF/ETRS89).'],['base_y','ECEF Y (m)','n'],
  ['base_z','ECEF Z (m)','n'],
  ['station_id','Station ID (0-4095)','n'],['itrf_year','ITRF realization year code (0-63)','n','0 = unspecified'],
  ['ind_gps','GPS indicator','c'],['ind_glo','GLONASS indicator','c'],['ind_gal','Galileo indicator','c'],
  ['gen1005','Generate 1005 on the ESP32','c','The LG290P does not send 1005; the ESP32 builds it from the position above.'],['gen1005_s','1005 interval (s)','n']]],
['GNSS UART',[['gnss_baud','Baud rate','n'],['gnss_rx','RX pin (restart needed)','n'],['gnss_tx','TX pin (-1 = none)','n']]],
['NTRIP upload 1',ntrip('n1_')],['NTRIP upload 2',ntrip('n2_')],
['NTRIP caster (local)',[['caster_en','Enabled','c'],['caster_port','Port','n'],['caster_mount','Mountpoint','t'],['caster_user','User','t','User and password both empty = open caster.'],
  ['caster_pass','Password','p'],['caster_max','Max clients','n'],['caster_timeout','Client stall timeout (s)','n']]],
['NTP server',[['ntp_en','Enabled','c'],['ntp_port','UDP port','n'],['ntp_lat_ms','NMEA latency compensation (ms)','n','Used only without 1PPS. Calibrate against a reference.']]],
['USB / logging',[['usb_mode','USB serial mode','s',[['0','Diagnostic console'],['1','Raw GNSS stream (PPP/PPK)']]],['usb_baud','Stream baud rate','n'],
  ['log_level','Log level','s',[['0','Error'],['1','Warn'],['2','Info'],['3','Debug']]]]]
];
function ntrip(p){return[[p+'en','Enabled','c'],[p+'host','Host','t'],[p+'port','Port','n'],[p+'mount','Mountpoint','t'],
  [p+'ver','NTRIP version','s',[['1','1 (SOURCE)'],['2','2 (POST)']]],[p+'user','User (v2 only)','t'],[p+'pass','Password (blank = keep)','p'],
  [p+'tls','TLS (not authenticated)','c','Encrypts the link but does not verify the server certificate.'],[p+'retry','Reconnect interval (s)','n']]}
function field(f){
  var k=f[0],t=f[2],h='<div class="row"><label for="'+k+'">'+esc(f[1])+'</label>';
  if(t==='c')h+='<input type="checkbox" id="'+k+'" name="'+k+'">';
  else if(t==='s')h+='<select id="'+k+'" name="'+k+'">'+f[3].map(function(o){return'<option value="'+o[0]+'">'+esc(o[1])+'</option>'}).join('')+'</select>';
  else h+='<input id="'+k+'" name="'+k+'" type="text"'+(t==='n'?' inputmode="decimal"':'')+(t==='p'?' autocomplete="off" autocapitalize="off" spellcheck="false"':'')+'>';
  if(t!=='s'&&f[3])h+='<small>'+esc(f[3])+'</small>';
  return h+'</div>';
}
function buildForm(c){
  var h='';
  SCHEMA.forEach(function(s){h+='<fieldset><legend>'+esc(s[0])+'</legend>'+s[1].map(field).join('')+'</fieldset>'});
  var rt=Object.keys(c).filter(function(k){return k.indexOf('rt_')===0});
  h+='<fieldset><legend>RTCM messages forwarded (from LG290P)</legend><p class="n">Selecting a message only <b>forwards</b> it if the receiver actually sends it. It does not make the LG290P output it. Message 1005 is generated by the ESP32 (see above).</p><div class="checks">'+
    rt.map(function(k){var n=k.slice(3);return'<label><input type="checkbox" name="'+k+'"> '+n+' <span class="n">'+esc(names[n]||'')+'</span></label>'}).join('')+'</div></fieldset>';
  var lg=Object.keys(c).filter(function(k){return k.indexOf('lg_')===0});
  h+='<fieldset><legend>Messages streamed to USB (PPP/PPK)</legend><p class="n">Forwarded only in "Raw GNSS stream" mode and only when actually received. RXM-RAWX / RXM-SFRBX are u-blox UBX messages; the LG290P does not normally emit them.</p><div class="checks">'+
    lg.map(function(k){return'<label><input type="checkbox" name="'+k+'"> '+esc(k.slice(3))+'</label>'}).join('')+'</div></fieldset>';
  $('#f').innerHTML=h;
}
function fill(c){
  Array.prototype.forEach.call($('#f').elements,function(e){
    if(!(e.name in c))return;
    if(e.type==='checkbox')e.checked=!!c[e.name];else e.value=c[e.name];
  });
}
function loadCfg(){
  return(Object.keys(names).length?Promise.resolve():api('/api/status').then(function(s){if(s.rtcm)s.rtcm.messages.forEach(function(m){names[m.msg]=m.name})}))
  .then(function(){return api('/api/config')}).then(function(c){buildForm(c);fill(c)});
}
function msg(el,t,ok){el.textContent=t;el.className=ok?'ok':'bad'}
function submit(save){
  var b=[];Array.prototype.forEach.call($('#f').elements,function(e){
    if(!e.name)return;b.push(encodeURIComponent(e.name)+'='+encodeURIComponent(e.type==='checkbox'?(e.checked?'1':'0'):e.value));
  });
  msg($('#msg'),'Applying…',1);
  api('/api/config?save='+(save?1:0),{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b.join('&')}).then(function(r){
    if(!r.ok)return msg($('#msg'),'Rejected: '+(r.error||r._s),0);
    var t='Applied live.';
    if(r.saveRequested)t=r.saved?'Applied and saved to flash.':'Applied, but SAVING FAILED.';else t+=' Not saved - use "Apply & Save" to keep after reboot.';
    if(r.restart)t+=' A restart is needed for the UART pin change.';
    msg($('#msg'),t,!r.saveRequested||r.saved);
  }).catch(function(){msg($('#msg'),'Request failed (Wi-Fi settings may have changed the connection).',0)});
}

/* ---------- wiring ---------- */
$('#tabs').addEventListener('click',function(e){
  var t=e.target.getAttribute('data-t');if(!t)return;tab=t;
  Array.prototype.forEach.call(document.querySelectorAll('#tabs button'),function(b){b.className=b.getAttribute('data-t')===t?'on':''});
  ['dash','cfg','rx','fwp'].forEach(function(s){$('#'+s).hidden=(s!==t)});
  if(t==='cfg')loadCfg();if(t==='dash'||t==='rx')poll();
});
$('#bApply').onclick=function(){submit(false)};$('#bSave').onclick=function(){submit(true)};
$('#bReload').onclick=function(){loadCfg().then(function(){msg($('#msg'),'Reloaded from device.',1)})};
$('#bRestart').onclick=function(){if(confirm('Restart the device? RTCM output stops for ~10 s.'))api('/api/restart',{method:'POST'}).then(function(){msg($('#msg'),'Restarting…',1)})};
$('#bFactory').onclick=function(){if(confirm('Erase ALL saved settings and restart with factory defaults?'))api('/api/factory',{method:'POST'}).then(function(r){msg($('#msg'),r.ok?'Reset; restarting with defaults…':'Failed: '+r.error,r.ok)})};
$('#bUp').onclick=function(){
  var f=$('#file').files[0],m=$('#upmsg');if(!f)return msg(m,'Choose a .bin file first.',0);
  var x=new XMLHttpRequest(),p=$('#prog');p.hidden=false;p.value=0;msg(m,'Uploading…',1);
  x.open('POST','/api/ota');x.upload.onprogress=function(e){if(e.lengthComputable)p.value=100*e.loaded/e.total};
  x.onload=function(){if(x.status===200){msg(m,'Upload verified. Device is restarting…',1);setTimeout(function(){location.reload()},15000)}else msg(m,'Failed: '+x.responseText+' (running firmware unchanged)',0)};
  x.onerror=function(){msg(m,'Connection lost during upload. Running firmware is unchanged; try again.',0)};
  x.send(f);
};
document.addEventListener('visibilitychange',poll);
poll();setInterval(poll,2000);
})();
