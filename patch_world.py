import re

file_path = "/home/jiang/workspace/fishbot/src/fishbot_description/world/jiang.world"
with open(file_path, "r") as f:
    data = f.read()

# Sizes
data = data.replace(
    "<link name='Wall_6'>\n        <collision name='Wall_6_Collision'>\n          <geometry>\n            <box>\n              <size>1.25 0.15 2.5</size>",
    "<link name='Wall_6'>\n        <collision name='Wall_6_Collision'>\n          <geometry>\n            <box>\n              <size>0.625 0.15 2.5</size>"
)
data = data.replace(
    "<visual name='Wall_6_Visual'>\n          <pose>0 0 1.25 0 -0 0</pose>\n          <geometry>\n            <box>\n              <size>1.25 0.15 2.5</size>",
    "<visual name='Wall_6_Visual'>\n          <pose>0 0 1.25 0 -0 0</pose>\n          <geometry>\n            <box>\n              <size>0.625 0.15 2.5</size>"
)
data = data.replace("<pose>-0.497902 -0.032516 0 0 -0 -1.5708</pose>", "<pose>-0.310402 -0.032516 0 0 -0 -1.5708</pose>")

data = data.replace(
    "<link name='Wall_8'>\n        <collision name='Wall_8_Collision'>\n          <geometry>\n            <box>\n              <size>1.25 0.15 2.5</size>",
    "<link name='Wall_8'>\n        <collision name='Wall_8_Collision'>\n          <geometry>\n            <box>\n              <size>0.625 0.15 2.5</size>"
)
data = data.replace(
    "<visual name='Wall_8_Visual'>\n          <pose>0 0 1.25 0 -0 0</pose>\n          <geometry>\n            <box>\n              <size>1.25 0.15 2.5</size>",
    "<visual name='Wall_8_Visual'>\n          <pose>0 0 1.25 0 -0 0</pose>\n          <geometry>\n            <box>\n              <size>0.625 0.15 2.5</size>"
)
data = data.replace("<pose>0.102098 -0.032516 0 0 -0 1.5708</pose>", "<pose>-0.085402 -0.032516 0 0 -0 1.5708</pose>")

data = data.replace(
    "<link name='Wall_7'>\n        <collision name='Wall_7_Collision'>\n          <geometry>\n            <box>\n              <size>0.75 0.15 2.5</size>",
    "<link name='Wall_7'>\n        <collision name='Wall_7_Collision'>\n          <geometry>\n            <box>\n              <size>0.375 0.15 2.5</size>"
)
data = data.replace(
    "<visual name='Wall_7_Visual'>\n          <pose>0 0 1.25 0 -0 0</pose>\n          <geometry>\n            <box>\n              <size>0.75 0.15 2.5</size>",
    "<visual name='Wall_7_Visual'>\n          <pose>0 0 1.25 0 -0 0</pose>\n          <geometry>\n            <box>\n              <size>0.375 0.15 2.5</size>"
)
data = data.replace("<pose>-0.197902 -0.582516 0 0 -0 0</pose>", "<pose>-0.197902 -0.270016 0 0 -0 0</pose>")

data = data.replace(
    "<link name='Wall_9'>\n        <collision name='Wall_9_Collision'>\n          <geometry>\n            <box>\n              <size>0.75 0.15 2.5</size>",
    "<link name='Wall_9'>\n        <collision name='Wall_9_Collision'>\n          <geometry>\n            <box>\n              <size>0.375 0.15 2.5</size>"
)
data = data.replace(
    "<visual name='Wall_9_Visual'>\n          <pose>0 0 1.25 0 -0 0</pose>\n          <geometry>\n            <box>\n              <size>0.75 0.15 2.5</size>",
    "<visual name='Wall_9_Visual'>\n          <pose>0 0 1.25 0 -0 0</pose>\n          <geometry>\n            <box>\n              <size>0.375 0.15 2.5</size>"
)
data = data.replace("<pose>-0.197902 0.517484 0 0 -0 3.14159</pose>", "<pose>-0.197902 0.204984 0 0 -0 3.14159</pose>")

# State
data = data.replace("<pose>-3.14332 -1.18211 0 0 0 -1.5708</pose>", "<pose>-2.95582 -1.18211 0 0 0 -1.5708</pose>")
data = data.replace("<pose>-2.54332 -1.18211 0 0 -0 1.5708</pose>", "<pose>-2.73082 -1.18211 0 0 -0 1.5708</pose>")
data = data.replace("<pose>-2.84332 -1.73211 0 0 -0 0</pose>", "<pose>-2.84332 -1.41961 0 0 -0 0</pose>")
data = data.replace("<pose>-2.84332 -0.632106 0 0 -0 3.14159</pose>", "<pose>-2.84332 -0.944606 0 0 -0 3.14159</pose>")

with open(file_path, "w") as f:
    f.write(data)

print("Patch complete")
