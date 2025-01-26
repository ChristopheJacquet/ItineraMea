extern crate quick_xml;

use clap::Parser;

use quick_xml::name::QName;
use quick_xml::Reader;
use quick_xml::events::Event;

use std::collections::HashMap;
use std::fs;
use std::str;

use std::path::{Path,PathBuf};

use std::time::Instant;

use walkdir::{DirEntry, WalkDir};

use rayon::prelude::*;

use regex::Regex;

#[derive(Parser, Debug)]
#[command(author, about = "Generate a tileset from GPX files.", long_about = None)]
struct Args {
    /// Input directory containing GPX files (walked recursively)
    #[arg(short, long, value_parser, num_args=1.., value_delimiter=',')]
    input_dir: Vec<PathBuf>,

    /// Output directory (for PNG tileset)
    #[arg(short, long)]
    output_dir: String,

    /// Enable tracing: will print out the name of the GPX files writing squares
    /// at the given coordinates, which must be of the form z,x,y where z is
    /// the zoom level (integer), x is the Mercator easting (integer), and y is
    /// the Mercator northing (integer). These are square coordinates, not tile
    /// coordinates (so the values are ~16 times larger when there are 16
    /// squares per tile).
    #[arg(long)]
    trace: Option<String>,
}

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

struct TraceInfo {
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
fn add_pixel_at_all_levels(mut xsq: i32, mut ysq: i32, tiles: &mut HashMap<TileKey, Tile>, gpx_path: &Path, trace_info: &Option<TraceInfo>) {
    for zoom in (0..=MAX_ZOOM).rev() {
        let (xtile, xdelta) = (xsq as u32 / NUM_SQUARES, xsq as u32 % NUM_SQUARES);
        let (ytile, ydelta) = (ysq as u32 / NUM_SQUARES, ysq as u32 % NUM_SQUARES);
        add_pixel(zoom, xtile, xdelta, ytile, ydelta, tiles);
        if let Some(t) = trace_info {
            if zoom == t.zoom && xsq as u32 == t.x && ysq as u32 == t.y {
                println!("Square set from {}", gpx_path.display());
            }
        }
        xsq /= 2;
        ysq /= 2;
    }
}

#[inline(always)]
fn bresenham(x0: i32, y0: i32, x1: i32, y1: i32, tiles: &mut HashMap<TileKey, Tile>, gpx_path: &Path, trace_info: &Option<TraceInfo>) {
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
        add_pixel_at_all_levels(pixx, pixy, tiles, gpx_path, trace_info);
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
    let mut contents = Vec::with_capacity(500);
    contents.extend_from_slice(b"\x89PNG\r\n\x1a\n");
    pack_png_chunk(b"IHDR", &[&(16 * NUM_SQUARES).to_be_bytes(), &(16 * NUM_SQUARES).to_be_bytes(), &[1, 3, 0, 0, 0]], &mut contents);
    pack_png_chunk(b"PLTE", &[&[0, 0, 0, 255, 0, 255]], &mut contents);
    pack_png_chunk(b"tRNS", &[&[0, 100]], &mut contents);
    pack_png_chunk(b"IDAT", &[&compressed], &mut contents); // TODO...
    pack_png_chunk(b"IEND", &[], &mut contents);

    // Try to read the destination file. If it exsists and its contents is the
    // same as the contents we have just generated, then abort (don't rewrite
    // it).
    let old_contents = std::fs::read(filename);
    if let Ok(c) = old_contents {
        if c == contents {
            return Ok(())
        }
    }

    std::fs::write(filename, &contents)
}

fn process_gpx(path: &Path, tiles: &mut HashMap<TileKey, Tile>, trace_info: &Option<TraceInfo>) {
    //println!("GPX: {}", path);
    let content = std::fs::read_to_string(path).ok().unwrap();

    let mut reader = Reader::from_str(&content);

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
                                QName(b"lat") => x = attribute_to_float(a),
                                QName(b"lon") => y = attribute_to_float(a),
                                _ => (),
                            }
                        }
                        let cur_sq = tile_num(x, y, MAX_ZOOM);
                        match prev_sq {
                            Some(s) => {
                                bresenham(s.x as i32, s.y as i32, cur_sq.x as i32, cur_sq.y as i32, tiles, path, trace_info);
                            },
                            _ => (),
                        }
                        prev_sq = Some(cur_sq);
                    },
                    _ => (),
                };
            },
            Ok(Event::Eof) => break, // exits the loop when reaching end of file
            Err(e) => panic!("Error at position {}: {:?}", reader.buffer_position(), e),
            _ => (), // There are several other `Event`s we do not consider here
        }
    }
}

fn is_file_with_extension(entry: &DirEntry, ext: &str) -> bool {
    entry.file_name()
        .to_str()
        .map(|s| s.ends_with(ext))
        .unwrap_or(false)
}

#[inline(never)]
fn read_gpx(input_dir: &Vec<PathBuf>, tiles: &mut HashMap<TileKey, Tile>, trace_info: &Option<TraceInfo>) {
    for dir in input_dir {
        println!("Processing input directory: {dir:?}.");
        let walker = WalkDir::new(dir).into_iter();

        for entry in walker {
            let e = entry.unwrap();
            if is_file_with_extension(&e, ".gpx") {
                //println!("{:?}", e.path().to_str().unwrap());
                process_gpx(&e.path(), tiles, trace_info);
            }
        }
    }
}

#[inline(never)]
fn write_tiles(output_dir: &str, tiles: &HashMap<TileKey, Tile>) {
    tiles.par_iter().for_each( |(key, tile)| {
        let _ = write_png_16(&tile, &format!("{}/{}/{}/{}.png", output_dir, key.zoom, key.x, key.y));
    });

    // Delete files for tiles that don't exist anymore.
    let walker = WalkDir::new(output_dir).into_iter();

    let tile_path_re: Regex = Regex::new(r"\/(\d+)\/(\d+)\/(\d+)\.png$").unwrap();

    for entry in walker {
        let e = entry.unwrap();
        if is_file_with_extension(&e, ".png") {
            let p = e.path().to_str().unwrap();
            match tile_path_re.captures(p) {
                Some(caps) => {
                    let key = TileKey {
                        zoom: caps[1].parse::<u32>().unwrap(),
                        x: caps[2].parse::<u32>().unwrap(),
                        y: caps[3].parse::<u32>().unwrap()
                    };
                    if ! tiles.contains_key(&key) {
                        let res = fs::remove_file(e.path());
                        match res {
                            Ok(()) => { println!("Deleted {}", p); }
                            Err(err) => { println!("Error: {}", err); }
                        }
                        
                    }
                }
                _ => {}
            }
        }
    }
}

fn main() {
    let args = Args::parse();

    let trace_info = match &args.trace {
        None => None,
        Some(t) => {
            let parts: Vec<&str> = t.split(",").collect();
            let int_parts: Vec<Result<u32, _>> = parts.into_iter().map(|x| x.parse::<u32>()).collect();
            match int_parts.as_slice() {
                [Ok(zoom), Ok(x), Ok(y)] => {
                    println!("Tracing square zoom={}, x={}, y={}", zoom, x, y);
                    Some(TraceInfo{
                        zoom: *zoom,
                        x: *x,
                        y: *y,
                    })
                },
                _ => {
                    println!("Bad format for --trace value. Expects z,x,y where z, x and y are unsigned integers.");
                    None
                }
            }
        },
    };

    let mut tiles: HashMap<TileKey, Tile> = HashMap::new();

    let start = Instant::now();

    read_gpx(&args.input_dir, &mut tiles, &trace_info);

    println!("==> At {}", start.elapsed().as_millis());

    println!("** Generating tiles **");

    write_tiles(&args.output_dir, &tiles);

    println!("==> At {}", start.elapsed().as_millis());
}
