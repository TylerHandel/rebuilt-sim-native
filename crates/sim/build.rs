// assets/constants.json (the JS game's js/constants.js, exported) as Rust constants, with the JS
// names as they are: FUEL.radius is FUEL::radius. Numbers are f64 (GROUP's bits u32), written in
// the shortest form that parses back to the same bits.
use serde_json::{Map, Value};
use std::fmt::Write;

fn num(x: f64) -> String {
    if x.is_infinite() { return if x > 0.0 { "f64::INFINITY".into() } else { "f64::NEG_INFINITY".into() }; }
    format!("{x:?}")
}

// the Rust type and value of a JSON value (None: arrays of mixed things, left out)
fn typed(name: &str, v: &Value, structs: &mut String) -> Option<(String, String)> {
    Some(match v {
        Value::Bool(b) => ("bool".into(), b.to_string()),
        Value::String(s) => ("&'static str".into(), format!("{s:?}")),
        Value::Number(n) => ("f64".into(), num(n.as_f64()?)),
        Value::Array(a) => {
            let items: Vec<_> = a.iter().enumerate().map(|(i, x)| typed(&format!("{name}_{i}"), x, structs)).collect::<Option<_>>()?;
            let t0 = items.first().map(|x| x.0.clone()).unwrap_or("f64".into());
            if items.iter().any(|x| x.0 != t0) { return None; }
            (format!("[{t0}; {}]", items.len()), format!("[{}]", items.iter().map(|x| x.1.clone()).collect::<Vec<_>>().join(", ")))
        }
        Value::Object(o) => {
            // (an object inside an array: a struct of its own)
            let sname = format!("{name}Item");
            let fields: Vec<_> = o.iter().map(|(k, x)| typed(&format!("{name}_{k}"), x, structs).map(|t| (k.clone(), t))).collect::<Option<_>>()?;
            if !structs.contains(&format!("pub struct {sname} ")) {
                writeln!(structs, "#[derive(Debug, Clone, Copy)] pub struct {sname} {{ {} }}", fields.iter().map(|(k, (t, _))| format!("pub {k}: {t}")).collect::<Vec<_>>().join(", ")).unwrap();
            }
            (sname.clone(), format!("{sname} {{ {} }}", fields.iter().map(|(k, (_, v))| format!("{k}: {v}")).collect::<Vec<_>>().join(", ")))
        }
        Value::Null => return None,
    })
}

fn object(name: &str, o: &Map<String, Value>, out: &mut String, structs: &mut String) {
    writeln!(out, "pub mod {name} {{\n    #[allow(unused_imports)] use super::*;").unwrap();
    for (k, v) in o {
        if name == "GROUP" { writeln!(out, "    pub const {k}: u32 = {};", v.as_u64().unwrap()).unwrap(); continue; }
        match v {
            Value::Object(inner) => object(k, inner, out, structs),
            _ => if let Some((t, val)) = typed(&format!("{name}_{k}"), v, structs) { writeln!(out, "    pub const {k}: {t} = {val};").unwrap(); },
        }
    }
    writeln!(out, "}}").unwrap();
}

fn main() {
    let path = "../../assets/constants.json";
    println!("cargo:rerun-if-changed={path}");
    let v: Value = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    let (mut out, mut structs) = (String::new(), String::new());
    for (k, v) in v["constants"].as_object().unwrap() {
        match v {
            Value::Object(o) => object(k, o, &mut out, &mut structs),
            _ => if let Some((t, val)) = typed(k, v, &mut structs) { writeln!(out, "pub const {k}: {t} = {val};").unwrap(); },
        }
    }
    std::fs::write(format!("{}/constants.rs", std::env::var("OUT_DIR").unwrap()), structs + &out).unwrap();
}
