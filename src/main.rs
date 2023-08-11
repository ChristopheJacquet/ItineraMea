extern crate quick_xml;

use quick_xml::name::QName;
use quick_xml::Reader;
use quick_xml::events::Event;

use std::collections::HashMap;
use std::str;

//use std::fs::File;
//use std::io::prelude::*;
//use std::path::Path;
use std::time::Instant;

use walkdir::{DirEntry, WalkDir};

struct Coords {
    x: u32,
    y: u32,
}

#[derive(PartialEq, Eq, Hash)]
struct TileKey {
    zoom: u32,
    x: u32,
    y: u32,
}

const NUM_SQUARES: u32 = 16;
const MAX_ZOOM: u32 = 16;
const CRC32: crc::Crc<u32> = crc::Crc::<u32>::new(&crc::CRC_32_ISO_HDLC);

type Tile = [[bool; NUM_SQUARES as usize]; NUM_SQUARES as usize];

#[inline(always)]
fn tile_num(lat_deg: f32, lon_deg: f32, zoom: u32) -> Coords {
    let lat_rad = lat_deg.to_radians();
    let n = 2_i32.pow(zoom) as f32;
    let xtile = (lon_deg + 180.0) / 360.0 * n;
    let ytile = (1.0 - (lat_rad.tan()).asinh() / std::f32::consts::PI) / 2.0 * n;
    return Coords {
        x: (xtile * (NUM_SQUARES as f32)) as u32,
        y: (ytile * (NUM_SQUARES as f32)) as u32,
    };
}

#[inline(always)]
fn add_pixel(zoom: u32, xtile: u32, xsq: u32, ytile: u32, ysq: u32, tiles: &mut HashMap<TileKey, Tile>) {
    let key = TileKey {
        zoom: zoom,
        x: xtile,
        y: ytile,
    };
    let tile: &mut Tile = tiles.entry(key).or_insert_with(|| [[false; NUM_SQUARES as usize]; NUM_SQUARES as usize]);
    tile[ysq as usize][xsq as usize] = true;
}

#[inline(always)]
fn add_pixel_at_all_levels(mut xsq: i32, mut ysq: i32, tiles: &mut HashMap<TileKey, Tile>) {
    for zoom in (0..=MAX_ZOOM).rev() {
        let (xtile, xdelta) = (xsq as u32 / NUM_SQUARES, xsq as u32 % NUM_SQUARES);
        let (ytile, ydelta) = (ysq as u32 / NUM_SQUARES, ysq as u32 % NUM_SQUARES);
        add_pixel(zoom, xtile, xdelta, ytile, ydelta, tiles);
        //if zoom == 16 && xtile == 35345 {
        //    println!("Expected tile!!!");
        //}
        xsq /= 2;
        ysq /= 2;
    }
}

#[inline(always)]
fn bresenham(x0: i32, y0: i32, x1: i32, y1: i32, tiles: &mut HashMap<TileKey, Tile>) {
    // Yield integer coordinates on the line from (x0, y0) to (x1, y1).

    // Input coordinates should be integers.

    // The result will contain both the start and the end point.

    // Source: https://pypi.org/project/bresenham

    let diffx = x1 - x0;
    let diffy = y1 - y0;

    let xsign = if diffx > 0 { 1 } else { -1 };
    let ysign = if diffy > 0 { 1 } else { -1 };

    let mut dx = diffx.abs();
    let mut dy = diffy.abs();

    let xx;
    let xy;
    let yx;
    let yy;

    if dx > dy {
        (xx, xy, yx, yy) = (xsign, 0, 0, ysign);
    } else {
        (dx, dy) = (dy, dx);
        (xx, xy, yx, yy) = (0, ysign, xsign, 0);
    }

    let mut d = 2*dy - dx;

    let mut pixx = x0;
    let mut pixy = y0;

    for _x in 0..=dx {
        //println!("Pixel: {:?}, {:?}", x0 + x*xx + y*yx, y0 + x*xy + y*yy);
        //naive: addPixelAtAllLevels(x0 + x*xx + y*yx, y0 + x*xy + y*yy, tiles);
        add_pixel_at_all_levels(pixx, pixy, tiles);
        //yield x0 + x*xx + y*yx, y0 + x*xy + y*yy
        if d >= 0 {
            d -= 2*dx;
            // optim
            pixx += yx;
            pixy += yy;
        }
        d += 2*dy;
        // optim
        pixx += xx;
        pixy += xy;
    }
}

#[inline(always)]
fn attribute_to_float(a: quick_xml::events::attributes::Attribute) -> f32 {
    return str::from_utf8(&a.value).unwrap().parse::<f32>().unwrap();
    // Note: this would be more correct, as attribute contents may be escaped:
    //    return a.unescape_value().unwrap().parse::<f32>().unwrap();
    // However, none of my GPX file seems to have escaped numeric attributes,
    // so not using unescape_value() is faster. The GPX analysis phase
    // goes from ~2680 ms to ~2480ms by removing unescape_value(), i.e. a ~7%
    // time decrease.
}

fn pack_png_chunk(png_tag: &[u8], data: &[&[u8]], contents: &mut Vec<u8>) {
    let mut data_size = 0_usize;
    for d in data {
        data_size += d.len();
    }
    contents.extend_from_slice(&(data_size as u32).to_be_bytes());
    contents.extend_from_slice(png_tag);
    for d in data {
        contents.extend_from_slice(d);
    }
    let checksum = CRC32.checksum(&contents[(contents.len()-data_size-4)..]);
    contents.extend_from_slice(&checksum.to_be_bytes());
}

fn write_png_16(tile: &Tile, filename: &str) -> std::io::Result<()> {
    let mut image = Vec::<u8>::with_capacity((16*NUM_SQUARES*(1+2*NUM_SQUARES)) as usize);
    for y in 0..NUM_SQUARES as usize {
        let mut line = Vec::<u8>::with_capacity(1+2*NUM_SQUARES as usize);
        line.push(0);
        for x in 0..NUM_SQUARES as usize {
            line.extend_from_slice(if tile[y][x]  { &[0xFF_u8, 0xFF_u8] } else { &[0_u8, 0_u8] });
        }
        for _ in 0..16 {
            image.extend_from_slice(&line);
        }
    }
    //println!("{:?}", &image);
    let compressed = miniz_oxide::deflate::compress_to_vec_zlib(&image, 9);
    let path = std::path::Path::new(filename);
    let prefix = path.parent().unwrap();
    std::fs::create_dir_all(prefix).unwrap();
    //let mut file = File::create(filename).unwrap();
    let mut contents = Vec::with_capacity(5000);
    contents.extend_from_slice(b"\x89PNG\r\n\x1a\n");
    pack_png_chunk(b"IHDR", &[&(16 * NUM_SQUARES).to_be_bytes(), &(16 * NUM_SQUARES).to_be_bytes(), &[1, 3, 0, 0, 0]], &mut contents);
    pack_png_chunk(b"PLTE", &[&[0, 0, 0, 255, 0, 255]], &mut contents);
    pack_png_chunk(b"tRNS", &[&[0, 100]], &mut contents);
    pack_png_chunk(b"IDAT", &[&compressed], &mut contents); // TODO...
    pack_png_chunk(b"IEND", &[], &mut contents);
    std::fs::write(filename, &contents)
    //file.write_all(&contents);
}

fn process_gpx(path: &str, tiles: &mut HashMap<TileKey, Tile>) {
    //println!("GPX: {}", path);
    let content = std::fs::read_to_string(path).ok().unwrap();

    let mut reader = Reader::from_str(&content);
    //reader.trim_text(true);

    let mut prev_sq: Option<Coords> = None;
    loop {
        match reader.read_event() {
            Ok(Event::Start(ref e)) => {
                match e.name() {
                    QName(b"trkseg") => {
                        prev_sq = None;
                    },
                    QName(b"trkpt") => {
                        let mut x: f32 = 0.;
                        let mut y: f32 = 0.;
                        for attr in e.attributes() {
                            let a = attr.unwrap();
                            match a.key {
                                QName(b"lat") => x = attribute_to_float(a), //println!("lat='{:?}'", attributeToFloat(a)), //String::from_utf8(a.value.to_vec())),
                                QName(b"lon") => y = attribute_to_float(a), //println!("lon='{:?}'", attributeToFloat(a)),
                                _ => (),
                            }
                        }
                        let cur_sq = tile_num(x, y, MAX_ZOOM);
                        match prev_sq {
                            Some(s) => {
                                bresenham(s.x as i32, s.y as i32, cur_sq.x as i32, cur_sq.y as i32, tiles);
                            },
                            _ => (),
                        }
                        prev_sq = Some(cur_sq);
                    },
                // String::from_utf8(
                    //println!("trkpt, attributes values: {:?}",
                                                            //e.attributes().map(|a| String::from_utf8(a.unwrap().value.to_vec()).unwrap()).collect::<Vec<_>>()),
                    _ => (),
                };
            },
            //Ok(Event::Text(e)) => println!("{:?}", e.unescape_and_decode(&reader).unwrap()),
            Ok(Event::Eof) => break, // exits the loop when reaching end of file
            Err(e) => panic!("Error at position {}: {:?}", reader.buffer_position(), e),
            _ => (), // There are several other `Event`s we do not consider here
        }
    }
}

fn is_gpx(entry: &DirEntry) -> bool {
    entry.file_name()
        .to_str()
        .map(|s| s.ends_with(".gpx"))
        .unwrap_or(false)
}

#[inline(never)]
fn read_gpx(tiles: &mut HashMap<TileKey, Tile>) {
    //let mut paths: Vec<String> = vec![];
    let dirs = vec!["/Users/cj/Documents/P/GPX_Archive/Final/",
    "/Users/cj/Documents/P/GPX_Archive/Quotidiennes/", "/Users/cj/Documents/P/GPX_Archive/PetitesVisites/"];

    for dir in dirs {
        let walker = WalkDir::new(dir).into_iter();

        for entry in walker {
            let e = entry.unwrap();
            if is_gpx(&e) {
                //println!("{:?}", e.path().to_str().unwrap());
                process_gpx(&e.path().to_str().unwrap(), tiles);
            }
        }
    }
    
    /*
    for entry in walker { //}.filter_entry(|e| isGPX(e)) {
        let e = &entry.unwrap();
        if isGPX(&e) {
            paths.push(e.path().to_str().unwrap().to_string());
        }
    }

    //paths.par_iter().for_each(|path| {
    for path in paths {
        println!("{:?}", path);
        processGPX(&path, &mut tiles);
    }
    */

}

#[inline(never)]
fn write_tiles(tiles: &HashMap<TileKey, Tile>) {
    for (key, tile) in tiles {
        let _ = write_png_16(&tile, &format!("/tmp/historando/{}/{}/{}.png", key.zoom, key.x, key.y));
        //println!("{:?}", key);
        // "/Users/cj/Documents/P/WebMaps/html/historando/
    }
}

fn main() {
    let mut tiles: HashMap<TileKey, Tile> = HashMap::new();

    let start = Instant::now();

    read_gpx(&mut tiles);

    println!("==> At {}", start.elapsed().as_millis());

    println!("** Generating tiles **");

    write_tiles(&tiles);

    println!("==> At {}", start.elapsed().as_millis());
}
