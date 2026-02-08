---
title: "Downloading Offline Map"
---
# Mobile Atlas Creator (MOBAC)

Mobile Atlas Creator is a tool used to download the tiles from a map tile API. In our case, we will attempt to download the tiles of the google maps satellite view map.

## Installing MOBAC

MOBAC can be downloaded [here](https://mobac.sourceforge.io/), the install is portable, meaning that all you have to do is click the download button and extract it to a folder somewhere to "install" it.

## Google Maps XML

MOBAC uses config files in XML format to set up custom tile stores to download from. In our case, we want to use google maps satellite view. In the directory that you extracted MOBAC to, there is a directory called mapsources, in that directory create a file called google.xml with the following contents:

```xml
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<customMultiLayerMapSource>
  <name>Google-hybridni-256</name>
  <tileType>jpg</tileType>
  <layers>
    <customMapSource>
      <name>Google-satelite-256</name>
      <minZoom>0</minZoom>
      <maxZoom>19</maxZoom>
      <tileType>jpg</tileType>
      <tileUpdate>None</tileUpdate>
      <url
        >http://mt{$serverpart}.google.com/vt/lyrs=s&amp;x={$x}&amp;y={$y}&amp;z={$z}&amp;scale=1</url
      >
      <serverParts>0 1 2 3</serverParts>
      <backgroundColor>#000000</backgroundColor>
    </customMapSource>
    <customMapSource>
      <name>Google-hybrid-256</name>
      <minZoom>0</minZoom>
      <maxZoom>19</maxZoom>
      <tileType>png</tileType>
      <tileUpdate>None</tileUpdate>
      <url
        >http://mt{$serverpart}.google.com/vt/lyrs=h&amp;x={$x}&amp;y={$y}&amp;z={$z}&amp;scale=1</url
      >
      <serverParts>0 1 2 3</serverParts>
    </customMapSource>
  </layers>
</customMultiLayerMapSource>
```

This will allow you to download google maps tiles from the google satellite maps API

## Using MOBAC

Now that your map source is configured, it is time to open MOBAC. This can be done by running the start.sh script in the MOBAC directory in your terminal. You will be met with this screen:

![image](https://github.com/umrover/mrover-ros/assets/71604997/b01eb37a-2c12-4288-8204-86df68979b03)

In the top left is your selection for which map tile API you would like to download from select the "Google-hybridni-256" option that you just created the config for. Zoom levels is used to selected which zoom levels of tiles you would like to download and also shows you how many individuals tiles (downloaded as .png images) will be used for your tile store. Also note that zoom levels are in ascending order, meaning that 19 is the zoom level that is the furthest zoomed in. Atlas content displays the current atlases that you will be creating. At the bottom left there is the "Create Atlas" button, which will allow you to download the selected area on the map.

You can navigate the map by right clicking and dragging, and using the zoom bar in the top left of the map to adjust which tile zoom level you are viewing.

To download your map store, first click "New" under Atlas Content. This will open you a menu like this:

![image](https://github.com/umrover/mrover-ros/assets/71604997/3f0c0f91-997f-4899-8805-467550449658)

Name it whatever you would like ("URC Map" or "CIRC Map" are good options), and then select "Osmdroid ZIP" from the list of formats. This format will give us the directory structure that we want for our offline atlas. Now drag an area around the part of the map you want to download and select which zoom levels you would like. Note that file size can grow rather quickly with higher zoom levels since we are using very detailed satellite map data, so try to focus in on the competition area as much as possible.

Once you have a box around the area you want, highlight your atlas in the Atlas Content menu and click "Add Selection." After this point click "Create Atlas" at the bottom and wait for it to download. This can take a while due to both file size and file number so try to do it in advance of competitions when you have a strong internet connection!

![image](https://github.com/umrover/mrover-ros/assets/71604997/2fbf96fd-328c-479b-8587-b30a8c8081ae)

After the download is complete you can click "Open Atlas Folder" and you will find a zip file with your newly download map tile store!

## Setting up the offline map for the GUI

The map tiles should be place in the src/teleoperation/frontend/public/map/[urc wc circ] directory of our code base. The [urc wc circ] folder should just contained the numbered folders, like so:

![image](https://github.com/umrover/mrover-ros/assets/71604997/4a69674d-9342-4cc8-9478-a3567a9e4aee)

After this point there is one small change that needs to be made in order for the offline map to work properly, and that is telling leaflet what the minimum available tile size is so that it know when to stop looking for new tiles. This will need to done in the offlineTileOptions of AutonRoverMap.vue and BasicRoverMap.vue. It should look something like this:

```js
const offlineTileOptions = {
  maxNativeZoom: 16,
  maxZoom: 100
};
```

In this case, if zoom level 16 were the most detailed we were to have downloaded this tells leaflet that after zoom level 16 it will zoom further into the level 16 zoom tiles instead of trying to find more detailed tiles if the zoom is increased.

With this change you have successfully set up the Base Station GUI map to be used without an internet connection! Just uncheck the online map box on any of the GUIs to test it out.

# S3 Storage
There exists an AWS S3 bucket that we used in the past to store the offline map. This has not been utilized in some time, but if you would like to use it to store your offline map tile store (so that we don't further push our google drive limit), reach out to any of the software 
leads to log into our team AWS account. If your map store is in the bucket it can be downloaded and automatically extracted to the static/map directory by running src/teleop/download_map/download_map.py after inputting our AWS keys to the json that the script automatically creates for you.