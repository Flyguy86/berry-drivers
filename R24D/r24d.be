# Seeedstudio MR24HPC1 / MicRadar R24DVD1 24Ghz mmWave radar Tasmota driver
# Source: https://github.com/blakadder/berry-drivers
# Tasmota driver written in Berry | code by blakadder (GPL-3.0)
# Edits: strict-mode fixes, live Presence/Activity/Motion publish, R24DState cmd

import string
import mqtt
import json

var topic = tasmota.cmd('Status ', true)['Status']['Topic']

# --- BEGIN: live MQTT state helper (keeps SENSOR in sync immediately) ---
var _r24d = { "Presence":"Unknown", "Activity":"None", "Motion":"None", "Body Movement Parameter":0 }

def _r24d_publish()
  tasmota.publish_sensor({ "R24DVD1": { "Human": _r24d } })
end

def _r24d_set_opt(presence, activity, motion, bmp)
  # Update fields if provided (use nil to skip)
  if presence != nil _r24d["Presence"] = presence end
  if activity != nil _r24d["Activity"] = activity end
  if motion   != nil _r24d["Motion"]   = motion   end
  if bmp      != nil _r24d["Body Movement Parameter"] = bmp end

  # Derive presence: consider Active/Still or bmp>0 as Occupied
  var act = _r24d["Activity"]
  var ibmp = int(_r24d["Body Movement Parameter"])
  if (act == "Active") || (act == "Still") || (ibmp > 0)
    _r24d["Presence"] = "Occupied"
  end
  _r24d_publish()
end

# Handy query command
tasmota.add_cmd("R24DState", def(cmd,idx,p,pj) tasmota.resp_cmnd_done_json({"R24DVD1":{"Human":_r24d}}) end)
# --- END: live MQTT state helper ---

class micradar : Driver

  static sensorname = "R24DVD1"
  static buffer = {}
  static cfg_buffer = {}
  static op_buffer = {}
  static header = bytes("5359")
  static endframe = "5443"
  static opbool

  static unk = "Unknown"
  static wok = { 0x0F: "OK" }

  static wactivity = { 0x00: "None", 0x01: "Still", 0x02: "Active" }
  static wduration = { 0x00:"0s",0x01:"10s",0x02:"30s",0x03:"1m",0x04:"2m",0x05:"5m",0x06:"10m",0x07:"30m",0x08:"60m" }
  static winitstatus = { 0x00:"Complete",0x01:"Incomplete",0x0F:"Completed" }
  static wmovement = { 0x00:"None",0x01:"Approaching",0x02:"Leaving" }
  static woccupancy = { 0x00:"Unoccupied",0x01:"Occupied" }
  static wscenemode = { 0x00:"Not Set",0x01:"Living Room",0x02:"Bedroom",0x03:"Bathroom",0x04:"Area Detection" }
  static wsensitivity = { 0x00:"None",0x01:"2m",0x02:"3m",0x03:"4m" }  # default 4m
  static wprotocolmode = { 0x00:"Standard",0x01:"Advanced" }
  static wbool = { 0x00:false, 0x01:true }
  static wonoff = { 0x00:"Off", 0x01:"On" }

  static word = {
    0x01: { "name":"System", "word": {
      0x01: { "name":"Heartbeat", "properties":micradar.wok },
      0x02: { "name":"Reset",     "properties":micradar.wok }
    }},
    0x02: { "name":"Information", "word": {
      0xA1:{ "name":"Product Model" },
      0xA2:{ "name":"Product ID" },
      0xA3:{ "name":"Hardware Model" },
      0xA4:{ "name":"Firmware Version" },
      0xA5:{ "name":"Protocol Type", "properties":micradar.wprotocolmode }
    }},
    0x05: { "name":"Status", "word": {
      0x01:{ "name":"Initialization", "properties":micradar.winitstatus },
      0x07:{ "name":"Scene Mode",     "properties":micradar.wscenemode },
      0x08:{ "name":"Sensitivity",    "properties":micradar.wsensitivity }
    }},
    0x80: { "name":"Human", "word": {
      0x01:{ "name":"Presence",               "properties":micradar.woccupancy },
      0x02:{ "name":"Activity",               "properties":micradar.wactivity },
      0x03:{ "name":"Body Movement Parameter" },
      0x0A:{ "name":"Unoccupied Delay",       "properties":micradar.wduration, "config":true },
      0x0B:{ "name":"Motion",                 "properties":micradar.wmovement }
    }},
    0x08: { "name":"Open Function", "word": {
      0x00:{ "name":"Switch", "properties":micradar.wonoff, "config":true },
      0x01:{ "name":"Report", "properties":["Static Energy","Static Distance","Motion Energy","Motion Distance","Movement Speed"] },
      0x06:{ "name":"Motion", "properties":micradar.wmovement },
      0x07:{ "name":"Body Movement Parameter" },
      0x08:{ "name":"Presence Energy Threshold" },
      0x09:{ "name":"Motion Amplitude Trigger Threshold", "config":true },
      0x0A:{ "name":"Presence Distance", "config":true },
      0x0B:{ "name":"Motion Distance",   "config":true },
      0x0C:{ "name":"Motion Trigger Time", "config":true },
      0x0C:{ "name":"Motion to Rest Time", "config":true },
      0x0D:{ "name":"Unoccupied State Time", "config":true }
    }}
  }

  var ser  # serial port

  def write2buffer(l, target)
    target.insert(l.find("name"), l.find("properties") != nil ? l["properties"][0x00] : 0)
  end

  def buffer_init()
    # Config bucket: only for 0x05 words flagged with "config"
    for k : self.word.keys()
      if k == 0x05
        self.cfg_buffer.insert(self.word[k].find("name"), {})
        for l : self.word[k]["word"]
          self.write2buffer(l, self.cfg_buffer[self.word[k]["name"]])
        end
      end
    end
    # Main buffer: all word groups > 0x7F (e.g., 0x80 Human)
    for k : self.word.keys()
      if k > 127
        self.buffer.insert(self.word[k].find("name"), {})
        for l : self.word[k]["word"]
          if l.find("config") != nil
            self.write2buffer(l, self.cfg_buffer[self.word[0x05]["name"]])
          else
            self.write2buffer(l, self.buffer[self.word[k]["name"]])
          end
        end
      end
    end
  end

  # Initialize serial (default TX/RX pins)
  def init(tx, rx)
    if !tx tx = gpio.pin(gpio.TXD) end
    if !rx rx = gpio.pin(gpio.RXD) end
    self.ser = serial(rx, tx, 115200, serial.SERIAL_8N1)
    tasmota.add_driver(self)
  end

  def restart()
    self.ser.write(self.encode("01", "02", "0F"))
    print("Reset command sent")
    tasmota.set_timer(3000, /-> self.get_config())
  end

  def publish2log(result, lvl)
    log(f"MicR: {result}", lvl == nil ? 3 : lvl)
  end

  def split_payload(b)
    var ret = []
    var s = size(b)
    var i = s - 2
    while i > 0
      if b[i] == 0x53 && b[i+1] == 0x59 && b[i-2] == 0x54 && b[i-1] == 0x43
        ret.insert(0, b[i..s-1])
        b = b[(0..i-1)]
      end
      i -= 1
    end
    ret.insert(0, b)
    return ret
  end

  def calculate_checksum(payload)
    var checksum = 0x00
    for i:0..size(payload)-1
      checksum = checksum + payload[i]
      checksum &= 0xFF
    end
    return checksum
  end

  def encode(ctrlword, cmndword, data)
    var d = bytes().fromhex(data)
    var b = self.header          # FIX: declare local 'b'
    b += bytes(ctrlword)
    b += bytes(cmndword)
    b.add(size(d), -2)
    b += d
    var chksum = self.calculate_checksum(b)
    b.add(chksum, 1)
    b += bytes(self.endframe)
    return b
  end

  def send(ctrlword, cmndword, data)
    var logr
    if !data data = "0F" end
    if size(ctrlword) != 2 && size(cmndword) != 2 && size(data) != 2
      logr = f"Parameters are wrong size!!! Must be in format: 00,00,00"
    else
      var payload_bin = self.encode(str(ctrlword), str(cmndword), str(data))
      self.ser.flush()
      self.ser.write(payload_bin)
      print("MicR: Sent =", str(payload_bin))
      logr = f"command payload {payload_bin} sent"
    end
    self.publish2log(logr, 2)
  end

  def id_data(msg)
    var prop = self.word[msg[2]]["word"][msg[3]].find("properties")
    var data = msg[6]
    var result = prop != nil ? prop.find(data) : data
    return result
  end

  def id_name(msg)
    return self.word[msg[2]]["word"][msg[3]].find("name", self.unk)
  end

  def id_cw(msg)
    return self.word[msg[2]].find("name", self.unk)
  end

  def get_config()
    self.send("08","00","0F")
    self.send("05","87","0F")
    self.send("05","88","0F")
    self.send("80","8A","0F")
  end

  def parse_productinfo(msg)
    var field = self.id_name(msg)
    var data  = msg[6..5+msg[5]].asstring()
    self.publish2log(f"{field}: {data}", 2)
  end

  def parse_message(msg)
    var field  = self.id_name(msg)
    var data   = self.id_data(msg)
    var cw     = self.id_cw(msg)
    var result = {}
    var val    = {}
    val.insert(field, data)
    result.insert(cw, val)

    # FIX: use actual variables (cw/field) instead of undefined a1/a2
    if self.buffer.find(cw) != nil
      if self.buffer[cw].find(field) != data
        self.buffer[cw].setitem(field, data)
        print(f"Buffer update {cw}: {field} with {data}")

        # Immediate per-field publish
        var pubtopic = "tele/" + topic + "/SENSOR"
        var mp = f"{{\"{self.sensorname}\":{json.dump(result)}}}"
        mqtt.publish(pubtopic, mp, false)
      end
    else
      self.publish2log(f"{field}: {data}", 2)
    end

    # --- NEW: keep live Human state in sync and derive Presence ---
    if cw == self.word[0x80]["name"]   # "Human"
      if field == "Activity"
        var act = str(data)
        var mot = (act == "Active" ? "Motion" : (act == "Still" ? "Micro" : "None"))
        _r24d_set_opt(nil, act, mot, nil)
      elif field == "Body Movement Parameter"
        _r24d_set_opt(nil, nil, nil, data)
      elif field == "Motion"
        _r24d_set_opt(nil, nil, str(data), nil)
      elif field == "Presence"
        _r24d_set_opt(str(data), nil, nil, nil)
      end
    end
  end

  def parse_config(msg)
    var field   = self.id_name(msg)
    var data    = self.id_data(msg)
    var cwname  = self.word[0x05]["name"]   # "Status"
    var result  = {}
    result.insert(field, data)

    if self.cfg_buffer.find(cwname) != nil
      if self.cfg_buffer[cwname].find(field)
        self.cfg_buffer[cwname].setitem(field, data)
        var pubtopic = "stat/" + topic + "/CONFIG"
        var mp = f"{{\"{self.sensorname}\":{json.dump(result)}}}"
        mqtt.publish(pubtopic, mp, false)
      end
    else
      self.publish2log(f"{field}: {data}", 2)
    end
  end

  def calc_distance(d)
    d = real(d) * 0.5
    return d
  end

  def parse_openprotocol(msg)
    # 0: Presence energy, 1: Static dist, 2: Motion energy, 3: Motion dist, 4: Speed
    var field = self.id_name(msg)
    var cw    = self.id_cw(msg)
    var data  = []
    var result = {}

    for i:6..5+msg[5]
      data.push(msg.get(i,1))
    end
    data.setitem(1, self.calc_distance(data[1]))
    data.setitem(3, self.calc_distance(data[3]))
    data.setitem(4, data[4] == 0 ? 0 : self.calc_distance(data[4] - 10))
    for i:0..size(data)-1
      result.insert(self.word[msg[2]]["word"][msg[3]]["properties"][i], data[i])
    end

    micradar.op_buffer = result
    self.publish2log(json.dump(result), 2)

    var pubtopic = "tele/" + topic + "/OPENPROTOCOL"
    mqtt.publish(pubtopic, json.dump(result), false)
  end

  def every_50ms()
    if self.ser.available() > 0
      var msg = self.ser.read()
      if size(msg) > 0
        if msg[0..1] == self.header
          var lst = self.split_payload(msg)
          for i:0..size(lst)-1
            msg = lst[i]
            if msg[2] == 0x02
              self.parse_productinfo(msg)
            else
              var cmndword = msg.get(3,1)
              if cmndword >= 128
                msg.set(3, (cmndword - 128), 1)
              end

              if msg[2] == 0x05 || self.word[msg[2]]['word'][msg[3]].find("config")
                self.parse_config(msg)
                if msg[3] == 0x01
                  self.get_config()
                end
              elif msg[2] == 0x08
                print("Open report received", msg)
                if msg[5] == 0x05
                  self.parse_openprotocol(msg)
                  if msg[3] == 0x00 self.opbool = msg[6] end
                else
                  self.parse_message(msg)
                end
              else
                self.parse_message(msg)
              end
            end
          end
        end
      end
    end
  end

  def json_append()
    var msg = f",\"{self.sensorname}\":{json.dump(self.buffer)}"
    tasmota.response_append(msg)
  end

  def web_sensor()
    if !self.ser return nil end
    var msg = []
    for k : self.buffer.keys()
      for l : self.buffer[k].keys()
        msg.push(f"{{s}}{l}{{m}}{self.buffer[k][l]}{{e}}")
      end
    end
    msg.push(f"{{s}}<i>Configuration Status{{m}}<HR>{{e}}")
    for k : self.cfg_buffer.keys()
      for l : self.cfg_buffer[k].keys()
        msg.push(f"{{s}}{l}{{m}}{self.cfg_buffer[k][l]}{{e}}")
      end
    end
    tasmota.web_send(msg.concat())
  end
end

radar = micradar()
tasmota.add_driver(radar)
radar.buffer_init()

# ----- Console commands -----

def set_scene(cmd, idx, payload, payload_json)
  payload = int(payload)
  var opt = [1,2,3,4]
  var ctl = "05"
  var cmw = "07"
  var val = "0F"
  if opt.find(payload) != nil
    val = f"{payload:.2i}"
  else
    cmw = "87"
    log("MicR: Set scene. Accepted value range is 1 - 4. No payload shows current configuration")
  end
  radar.send(ctl, cmw, val)
  tasmota.resp_cmnd_done()
end
tasmota.add_cmd('SetScene', set_scene)

def set_sensitivity(cmd, idx, payload, payload_json)
  var opt = [1,2,3]
  var ctl = "05"
  var cmw = "08"
  var val = "0F"
  if opt.find(int(payload)) != nil
    val = f"{payload:.2i}"
  else
    cmw = "88"
    log("MicR: Set sensitivity. Accepted value range is 1 - 3. No payload shows current configuration")
  end
  radar.send(ctl, cmw, val)
  tasmota.resp_cmnd_done()
end
tasmota.add_cmd('SetSensitivity', set_sensitivity)

def set_delay(cmd, idx, payload, payload_json)
  var ctrlword = "80"
  var cmndword = "0A"
  var val = "0F"
  if int(payload) <= 8 && int(payload) >= 0   # FIX: allow 0..8
    val = f"{payload:.2i}"
  else
    cmndword = int(cmndword) + 128
    log("MicR: Set unoccupancy delay. Accepted value range is 0 - 8. No payload shows current configuration")
  end
  radar.send(ctrlword, cmndword, val)
  tasmota.resp_cmnd_done()
end
tasmota.add_cmd('SetDelay', set_delay)

def radar_send(cmd, idx, payload, payload_json)
  var data = string.split(payload, ",")
  if size(data) < 3 data.push("0F") end
  radar.send(data[0], data[1], data[2])
  tasmota.resp_cmnd_done()
end
tasmota.add_cmd('RadarSend', radar_send)

def restart_cmnd(cmd, idx, payload, payload_json)
  radar.restart()
  tasmota.resp_cmnd_done()
end
tasmota.add_cmd('RadarRestart', restart_cmnd)

tasmota.add_rule("system#boot", /-> radar.restart() )  # restart radar on boot to populate sensors
