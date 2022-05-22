# 4BarLinkageVisualization
![Python 3.9](https://img.shields.io/badge/python-3.9-green.svg)

ESP32BoardManager Version 2.0.0
a

## วิธีรัน
1. เปิด GUI ขึ้นมา (GUI/mainui.py)
2. เสียบ ESP32 เข้าพอร์ท USB
3. เลือกพอร์ทใน GUI แล้วกด Connect
4. ถ้าค่าค้าง/ไม่แสดงอะไร ให้ปิด GUI แล้วเริ่มทำขั้นตอน 1 ใหม่

## วิธีใข้ Logging
1. กด Start Logging เพื่อเริ่ม Logging
2. กด Stop Logging เพื่อหยุด Logging

  ** ถ้ากด Stop แล้ว Start ใหม่โปรแกรมจะไปสร้างไฟล์ใหม่แล้วเก็บ Log ใส่ไฟล์นั้นแทน 
  
  ** format ชื่อไฟล์จะเป็น 'results' + timestamp + '.csv'

## บัคในโปรแกรม
ตอนนี้ยังมีบัคของการคุยกันระหว่างตัว UI กับ ESP อยู่ 
  -> วิธีแก้ปัญหาตอนนี้คือ รันตามขั้นตอนด้านบนไปก่อน
  
  ** ถ้าเจอบัคอะไรเพิ่มเติม สร้าง Issue ละเขียนรายละเอียดของบัคมาเลยครับ เดี๋ยวจะพยายามแก้ให้

## TODO

- [ ] https://github.com/phoori12/4BarLinkageVisualization/issues/1
