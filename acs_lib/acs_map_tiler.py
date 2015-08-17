#!/usr/bin/env python3
"""
    ACS Map Tiler
    Michael Day
    Nov 2014
"""
import threading, os, time, string, math
import urllib3
import hashlib

from collections import OrderedDict
from PIL import Image

from acs_lib import acs_math

TILES_WIDTH = 256
TILES_HEIGHT = 256

TILE_SERVICES = { "GoogleSat"      : "https://khm${GOOG_DIGIT}.google.com/kh/v=157&hl=pt-PT&x=${X}&y=${Y}&z=${ZOOM}&s=${GALILEO}",
    "GoogleMap"      : "https://mt${GOOG_DIGIT}.google.com/vt/lyrs=m@132&hl=pt-PT&x=${X}&y=${Y}&z=${ZOOM}&s=${GALILEO}",
    "GoogleTer"      : "https://mt${GOOG_DIGIT}.google.com/vt/v=t@132,r@249&hl=pt-PT&x=${X}&y=${Y}&z=${ZOOM}&s=${GALILEO}",
    "GoogleChina"    : "http://mt${GOOG_DIGIT}.google.cn/vt/lyrs=m@121&hl=en&gl=cn&x=${X}&y=${Y}&z=${ZOOM}&s=${GALILEO}",
    "YahooMap"       : "http://maps${Y_DIGIT}.yimg.com/hx/tl?v=4.3&.intl=en&x=${X}&y=${YAHOO_Y}&z=${YAHOO_ZOOM}&r=1",
    "YahooSat"       : "http://maps${Y_DIGIT}.yimg.com/ae/ximg?v=1.9&t=a&s=256&.intl=en&x=${X}&y=${YAHOO_Y}&z=${YAHOO_ZOOM}&r=1",
    "YahooInMap"     : "http://maps.yimg.com/hw/tile?locale=en&imgtype=png&yimgv=1.2&v=4.1&x=${X}&y=${YAHOO_Y}&z=${YAHOO_ZOOM_2}",
    "YahooInHyb"     : "http://maps.yimg.com/hw/tile?imgtype=png&yimgv=0.95&t=h&x=${X}&y=${YAHOO_Y}&z=${YAHOO_ZOOM_2}",
    "YahooHyb"       : "http://maps${Y_DIGIT}.yimg.com/hx/tl?v=4.3&t=h&.intl=en&x=${X}&y=${YAHOO_Y}&z=${YAHOO_ZOOM}&r=1",
    "MicrosoftBrMap" : "http://imakm${MS_DIGITBR}.maplink3.com.br/maps.ashx?v=${QUAD}|t&call=2.2.4",
    "MicrosoftHyb"   : "http://ecn.t${MS_DIGIT}.tiles.virtualearth.net/tiles/h${QUAD}.png?g=441&mkt=en-us&n=z",
    "MicrosoftSat"   : "http://ecn.t${MS_DIGIT}.tiles.virtualearth.net/tiles/a${QUAD}.png?g=441&mkt=en-us&n=z",
    "MicrosoftMap"   : "http://ecn.t${MS_DIGIT}.tiles.virtualearth.net/tiles/r${QUAD}.png?g=441&mkt=en-us&n=z",
        "MicrosoftTer"   : "http://ecn.t${MS_DIGIT}.tiles.virtualearth.net/tiles/r${QUAD}.png?g=441&mkt=en-us&shading=hill&n=z",
        "OviSat"         : "http://maptile.maps.svc.ovi.com/maptiler/v2/maptile/newest/satellite.day/${Z}/${X}/${Y}/256/png8",
        "OviHybrid"      : "http://maptile.maps.svc.ovi.com/maptiler/v2/maptile/newest/hybrid.day/${Z}/${X}/${Y}/256/png8",
    "OpenStreetMap"  : "http://tile.openstreetmap.org/${ZOOM}/${X}/${Y}.png",
    "OSMARender"     : "http://tah.openstreetmap.org/Tiles/tile/${ZOOM}/${X}/${Y}.png",
    "OpenAerialMap"  : "http://tile.openaerialmap.org/tiles/?v=mgm&layer=openaerialmap-900913&x=${X}&y=${Y}&zoom=${OAM_ZOOM}",
    "OpenCycleMap"   : "http://andy.sandbox.cloudmade.com/tiles/cycle/${ZOOM}/${X}/${Y}.png"
    }

# these are the md5sums of "unavailable" tiles
BLANK_TILES = set(["d16657bbee25d7f15c583f5c5bf23f50",
                   "c0e76e6e90ff881da047c15dbea380c7",
                   "d41d8cd98f00b204e9800998ecf8427e"])

class TileServiceInfo:
    '''a lookup object for the URL templates'''
    def __init__(self, x, y, zoom):
        self.X = x
        self.Y = y
        self.Z = zoom
        quadcode = ''
        for i in range(zoom - 1, -1, -1):
            quadcode += str((((((y >> i) & 1) << 1) + ((x >> i) & 1))))
        self.ZOOM = zoom
        self.QUAD = quadcode
        self.YAHOO_Y = 2**(zoom-1) - 1 - y
        self.YAHOO_ZOOM = zoom + 1
        self.YAHOO_ZOOM_2 = 17 - zoom + 1
        self.OAM_ZOOM = 17 - zoom
        self.GOOG_DIGIT = (x + y) & 3
        self.MS_DIGITBR = (((y & 1) << 1) + (x & 1)) + 1
        self.MS_DIGIT = (((y & 3) << 1) + (x & 1))
        self.Y_DIGIT = (x + y + zoom) % 3 + 1
        self.GALILEO = "Galileo"[0:(3 * x + y) & 7]

    def __getitem__(self, a):
        return str(getattr(self, a))

class TileInfo:
    '''description of a tile'''
    def __init__(self, tile, zoom, service, offset=(0,0)):
        self.tile = tile
        (self.x, self.y) = tile
        self.zoom = zoom
        self.service = service
        (self.offsetx, self.offsety) = offset
        self.refresh_time()

    def key(self):
        '''tile cache key'''
        return (self.tile, self.zoom, self.service)

    def refresh_time(self):
        '''reset the request time'''
        self.request_time = time.time()

    def coord(self, offset=(0,0)):
        '''return lat,lon within a tile given (offsetx,offsety)'''
        (tilex, tiley) = self.tile
        (offsetx, offsety) = offset
        world_tiles = 1<<self.zoom
        x = ( tilex + 1.0*offsetx/TILES_WIDTH ) / (world_tiles/2.) - 1
        y = ( tiley + 1.0*offsety/TILES_HEIGHT) / (world_tiles/2.) - 1
        lon = x * 180.0
        y = math.exp(-y*2*math.pi)
        e = (y-1)/(y+1)
        lat = 180.0/math.pi * math.asin(e)
        return (lat, lon)

    def size(self):
        '''return tile size as (width,height) in meters'''
        (lat1, lon1) = self.coord((0,0))
        (lat2, lon2) = self.coord((TILES_WIDTH,0))
        width = acs_math.gps_distance(lat1, lon1, lat2, lon2)
        (lat2, lon2) = self.coord((0,TILES_HEIGHT))
        height = acs_math.gps_distance(lat1, lon1, lat2, lon2)
        return (width,height)

    def distance(self, lat, lon):
        '''distance of this tile from a given lat/lon'''
        (tlat, tlon) = self.coord((TILES_WIDTH/2,TILES_HEIGHT/2))
        return acs_math.gps_distance(lat, lon, tlat, tlon)

    def path(self):
        '''return relative path of tile image'''
        (x, y) = self.tile
        return os.path.join('%u' % self.zoom,
                    '%u' % y,
                    '%u.img' % x)

    def url(self, service):
        '''return URL for a tile'''
        if service not in TILE_SERVICES:
            raise TileException('unknown tile service %s' % service)
        url = string.Template(TILE_SERVICES[service])
        (x,y) = self.tile
        tile_info = TileServiceInfo(x, y, self.zoom)
        return url.substitute(tile_info)


class TileInfoScaled(TileInfo):
    '''information on a tile with scale information and placement'''
    def __init__(self, tile, zoom, scale, src, dst, service):
        TileInfo.__init__(self, tile, zoom, service)
        self.scale = scale
        (self.srcx, self.srcy) = src
        (self.dstx, self.dsty) = dst

    def print(self):
        """print out what's in the TileInfoScaled object"""
        print("TileInfoScaled")
        print("\tTile:", self.tile)
        print("\tScale:", self.scale)


class ACS_MapTiler():
    def __init__(self, lat, lon, min_zoom=0, max_zoom=20, cache_path=None, cache_size=500, service='OviHybrid', refresh_age=30*24*60*60):
        self.__debug = False

        self.__current_tile_service = service
        self.__tile_fetching_thread = None
        self.__tile_delay = 0.1
        # A dictionary of TileInfo objects to attempt to download:
        self.__downloads_pending = {}
        self.__refresh_age=refresh_age

        #TODO: "Unavailable" and "Loading..." tiles
        #self.__unavailable = ACS_MapTiler.tile_img_from_file("/home/maday/test_tiling/unavailable.jpg");
        self.__unavailable = None
        self.__lat = lat 
        self.__lon = lon
        self.__max_zoom = max_zoom
        self.__min_zoom = min_zoom

        if cache_path is None:
            cache_path = os.path.join(os.environ['HOME'], '.tilecache')
        
        self.__cache_path = cache_path

        if not os.path.exists(self.__cache_path):
            os.makedirs(self.__cache_path)

        self.__cache_size = cache_size
        self.__service = service

        if service not in TILE_SERVICES:
            raise Exception('Unknown tile service %s' % service)

    #d is a boolean
    def set_debug(self, d):
        self.__debug = d

    def tile_img_from_file(file_path):
        #img = Image.open(file_path)
        #return img

        #couldn't figure out how to convert from PIL.Image to
        #raw bytes.  Punting for now and storing image in GUI library instead.
        #If I ever get more than one GUI library I will need to come back
        #and address this

        return None

    def tiles_pending(self):
        '''return number of tiles pending download'''
        return len(self.__downloads_pending)

    def coord_to_tile(self, lat, lon, zoom):
        '''convert lat/lon/zoom to a TileInfo'''
        world_tiles = 1<<zoom
        x = float(world_tiles) / 360.0 * (lon + 180.0)
        tiles_pre_radian = float(world_tiles) / (2.0 * math.pi)
        e = math.sin(lat * (1.0/180.0*math.pi))
        y = float(world_tiles)/2.0 + 0.5*math.log((1.0+e)/(1.0-e)) * (-tiles_pre_radian)

        offsetx = int((x - int(x)) * TILES_WIDTH)
        offsety = int((y - int(y)) * TILES_HEIGHT)
        return TileInfo((int(x) % world_tiles, int(y) % world_tiles), zoom, self.__service, offset=(offsetx, offsety))

    def area_to_tile_list_lat_lon(self, lat_top, lat_bottom, lon_left, lon_right, zoom):
        if lat_top >= 85.0:
            lat_top = 85.0
        if lat_bottom <= -85.0:
            lat_bottom = -85.0
        if lon_left <= -180.0:
            lon_left = -179.99999
        if lon_right >= 180.0:
            lon_right = 179.99999

        tile_min = self.coord_to_tile(lat_top, lon_left, zoom)
        tile_max = self.coord_to_tile(lat_bottom, lon_right, zoom)

        ret = []

        #put some TileInfos in a list and return them
        for y in range(tile_min.y, tile_max.y+1):
            for x in range(tile_min.x, tile_max.x+1):
                ret.append(TileInfo((x,y), zoom, self.__service))

        return ret

    #TODO: may be able to get rid of this method in favor of area_to_tile_list_lat_lon
    def area_to_tile_list(self, lat, lon, width, height, ground_width, zoom=None):
        #make sure we don't have ints where we dont want them
        ground_width = float(ground_width)

        pixel_width = ground_width / float(width)
        ground_height = ground_width * (float(height) / float(width))
        top_right = acs_math.gps_newpos(lat, lon, 90.0, ground_width)
        bottom_left = acs_math.gps_newpos(lat, lon, 180.0, ground_height)
        bottom_right = acs_math.gps_newpos(bottom_left[0], bottom_left[1], 90, ground_width)

        # choose a zoom level if not provided
        if zoom is None:
            zooms = range(self.__min_zoom, self.__max_zoom+1)
        else:
            zooms = [zoom]
        for zoom in zooms:
            tile_min = self.coord_to_tile(lat, lon, zoom)
            (twidth,theight) = tile_min.size()
            tile_pixel_width = twidth / float(TILES_WIDTH)
            scale = pixel_width / tile_pixel_width
            if scale >= 1.0:
                break
        
        scaled_tile_width = int(TILES_WIDTH / scale)
        scaled_tile_height = int(TILES_HEIGHT / scale)

        # work out the bottom right tile
        tile_max = self.coord_to_tile(bottom_right[0], bottom_right[1], zoom)

        ofsx = int(tile_min.offsetx / scale)
        ofsy = int(tile_min.offsety / scale)
        srcy = ofsy
        dsty = 0

        ret = []

        # put the tiles' info in a list and return it
        for y in range(tile_min.y, tile_max.y+1):
            srcx = ofsx
            dstx = 0
            for x in range(tile_min.x, tile_max.x+1):
                if dstx < width and dsty < height:
                    ret.append(TileInfoScaled((x,y), zoom, scale,
                        (srcx,srcy), (dstx,dsty), self.__service))
                dstx += scaled_tile_width-srcx
                srcx = 0
            dsty += scaled_tile_height-srcy
            srcy = 0
        return ret

    def tile_to_path(self, tile):
        '''return full path to a tile'''
        return os.path.join(self.__cache_path, self.__service, tile.path())

    def load_tile(self, tile):
        '''ask the tile to start downloading if not already downloading it'''

        key = tile.key()
        path = self.tile_to_path(tile)

        # if it is an old tile, then try to refresh
        modified_time = 0
        try:
            modified_time = os.path.getmtime(path)
        except Exception:
            pass               

        if modified_time + self.__refresh_age < time.time():
            try:
                self.__downloads_pending[key].refresh_time()
            except Exception:
                self.__downloads_pending[key] = tile
        
        #make sure we're trying to get tiles
        self.start_fetching_tiles()

    def fetch_tile_thread(self):
        '''thread that downloads tiles from the current source'''

        http = urllib3.PoolManager()

        while self.tiles_pending() > 0:
            time.sleep(self.__tile_delay)
        
            # work out which one to download next, choosing by request_time
            tile_info = None
            
            #make a copy of __downloads_pending since it might change during
            #this iteration:
            pending_downloads = self.__downloads_pending.copy()
            for key, next_tile in pending_downloads.items():
                if tile_info is None or next_tile.request_time > tile_info.request_time:
                    tile_info = next_tile

            url = tile_info.url(self.__service)
            path = self.tile_to_path(tile_info)
            key = tile_info.key()

            if self.__debug:
                print("Downloading %s [%u left]" % (url, self.tiles_pending()))
                
            try:
                r = http.request('GET', url)

                #TODO if I use google (need to convert from urllib2 to urllibe3)
                #if url.find('google') != -1:
                #    req.add_header('Referer', 'https://maps.google.com/')
                
                headers = r.headers

            except Exception as e:
                print('Error loading %s' % url)
                self.__downloads_pending.pop(key)
                if self.__debug:
                    print("Failed %s: %s" % (url, str(e)))
                
                continue

            if 'content-type' not in headers or headers['content-type'].find('image') == -1:
                self.__downloads_pending.pop(key)

                if self.__debug:
                    print("non-image response %s" % url)
                continue
            else:
                img = r.data

            #print(img)

            # see if its a blank/unavailable tile
            md5 = hashlib.md5(img).hexdigest()
            if md5 in BLANK_TILES:
                if self.__debug:
                    print("blank tile %s" % url)
                
                self.__downloads_pending.pop(key)
                continue

            os.makedirs(os.path.dirname(path), exist_ok=True)
            
            h = open(path+'.tmp','wb')
            h.write(img)
            h.close()
            try:
                os.unlink(path)
            except Exception:
                pass

            os.rename(path+'.tmp', path)
            self.__downloads_pending.pop(key)

        #all done with the fetching thread:
        self.__tile_fetching_thread = None

    def start_fetching_tiles(self):
        '''start the tile fetch thread'''

        # don't run more than one thread
        if self.__tile_fetching_thread != None:
            return

        self.__tile_fetching_thread = threading.Thread(target=self.fetch_tile_thread)
        self.__tile_fetching_thread.daemon = True
        self.__tile_fetching_thread.start()

    #TODO: add lat, lon, width, height, ground_size args for arbitrary location
    #TODO: may be able to get rid of this method in favor of prefetch_lat_lon
    def prefetch(self, width=1024, height=1024, ground_width=5000):
        for next_zoom in range(self.__min_zoom, self.__max_zoom+1):
            tile_info_list = self.area_to_tile_list(self.__lat, self.__lon, width, height, ground_width, next_zoom)
            tile_info_list[0].print()

        for next_tile_info in tile_info_list:
            self.load_tile(next_tile_info)

    def prefetch_lat_lon(self, lat_top, lat_bottom, lon_left, lon_right, min_zoom, max_zoom):
        #TODO: deal with the case where "left" to "right" cros the 
        #international date line

        for next_zoom in range(min_zoom, max_zoom+1):
            tile_info_list = self.area_to_tile_list_lat_lon(lat_top, lat_bottom,
                    lon_left, lon_right, next_zoom)

            for next_tile_info in tile_info_list:
                self.load_tile(next_tile_info)

    def set_max_zoom(self, zoom_level):
        self.__max_zoom = zoom_level

    def get_min_zoom(self):
        return self.__min_zoom

    def unload(self):
        pass
