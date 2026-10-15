//! Decoder for ROS 2 messages in plain CDR (as written by rmw_zenoh), driven by
//! parsed `.msg` definitions. The result is a JSON value, in field order.

use std::sync::Arc;

use serde_json::{Map, Number, Value};

use super::parse::{Arr, Base, MsgDef, Prim};

/// Primitive arrays longer than this are shown as `"<N x type>"`, not listed.
const MAX_LISTED: usize = 64;
/// Longest string shown, in characters.
const MAX_STRING_CHARS: usize = 1000;
/// Deepest nesting followed; guards against recursive definitions.
const MAX_DEPTH: usize = 32;

/// Decode `data` (with its 4-byte CDR header) as the message `root`.
/// `lookup` returns the definition of a message, or None while it is not loaded.
pub fn decode(
    data: &[u8],
    root: &str,
    lookup: &dyn Fn(&str) -> Option<Arc<MsgDef>>,
) -> Result<Value, String> {
    if data.len() < 4 || data[0] != 0 || data[1] > 1 {
        return Err("not plain CDR".into());
    }
    let mut r = Reader {
        data: &data[4..],
        pos: 0,
        le: data[1] == 1,
        lookup,
    };
    let value = r.message(root, 0)?;
    // CDR may pad the end; anything more means the definition does not fit.
    if r.data.len() - r.pos > 3 {
        return Err(format!("{} bytes left over", r.data.len() - r.pos));
    }
    Ok(value)
}

struct Reader<'a> {
    data: &'a [u8],
    pos: usize,
    le: bool,
    lookup: &'a dyn Fn(&str) -> Option<Arc<MsgDef>>,
}

impl Reader<'_> {
    fn align(&mut self, n: usize) -> Result<(), String> {
        self.pos = (self.pos + n - 1) & !(n - 1);
        if self.pos > self.data.len() {
            return Err("payload too short".into());
        }
        Ok(())
    }

    fn take(&mut self, n: usize) -> Result<&[u8], String> {
        let end = self.pos.checked_add(n).filter(|&e| e <= self.data.len());
        let end = end.ok_or("payload too short")?;
        let bytes = &self.data[self.pos..end];
        self.pos = end;
        Ok(bytes)
    }

    /// The next `N` bytes, aligned to `N`, as little-endian bytes.
    fn num<const N: usize>(&mut self) -> Result<[u8; N], String> {
        self.align(N)?;
        let mut a = [0u8; N];
        a.copy_from_slice(self.take(N)?);
        if !self.le {
            a.reverse();
        }
        Ok(a)
    }

    fn message(&mut self, name: &str, depth: usize) -> Result<Value, String> {
        if depth > MAX_DEPTH {
            return Err("nested too deep".into());
        }
        let def = (self.lookup)(name).ok_or_else(|| format!("definition of {name} not loaded"))?;
        if def.fields.is_empty() {
            self.take(1)?; // rosidl adds a dummy member to an empty message
        }
        let mut obj = Map::new();
        for field in &def.fields {
            let value = match field.arr {
                Arr::One => self.element(&field.base, depth)?,
                Arr::Fixed(n) => self.array(&field.base, n, depth)?,
                Arr::Seq => {
                    let n = u32::from_le_bytes(self.num::<4>()?) as usize;
                    self.array(&field.base, n, depth)?
                }
            };
            obj.insert(field.name.clone(), value);
        }
        Ok(Value::Object(obj))
    }

    fn array(&mut self, base: &Base, n: usize, depth: usize) -> Result<Value, String> {
        let min = match base {
            Base::Prim(p) => p.size(),
            Base::Nested(_) => 1,
        };
        if n.checked_mul(min).is_none_or(|b| b > self.data.len() - self.pos) {
            return Err("array longer than payload".into());
        }
        if let Base::Prim(p) = base {
            if *p != Prim::Str && n > MAX_LISTED {
                self.align(p.size())?;
                self.take(n * p.size())?;
                return Ok(Value::String(format!("<{n} x {}>", p.name())));
            }
        }
        let mut items = Vec::with_capacity(n);
        for _ in 0..n {
            items.push(self.element(base, depth)?);
        }
        Ok(Value::Array(items))
    }

    fn element(&mut self, base: &Base, depth: usize) -> Result<Value, String> {
        match base {
            Base::Nested(name) => self.message(name, depth + 1),
            Base::Prim(p) => self.prim(*p),
        }
    }

    fn prim(&mut self, p: Prim) -> Result<Value, String> {
        Ok(match p {
            Prim::Bool => Value::Bool(self.num::<1>()?[0] != 0),
            Prim::U8 => self.num::<1>()?[0].into(),
            Prim::I8 => i8::from_le_bytes(self.num::<1>()?).into(),
            Prim::U16 => u16::from_le_bytes(self.num::<2>()?).into(),
            Prim::I16 => i16::from_le_bytes(self.num::<2>()?).into(),
            Prim::U32 => u32::from_le_bytes(self.num::<4>()?).into(),
            Prim::I32 => i32::from_le_bytes(self.num::<4>()?).into(),
            Prim::U64 => u64::from_le_bytes(self.num::<8>()?).into(),
            Prim::I64 => i64::from_le_bytes(self.num::<8>()?).into(),
            Prim::F32 => {
                let x = f32::from_le_bytes(self.num::<4>()?);
                // Through the shortest f32 text, so 0.1 stays 0.1.
                float(x.to_string().parse().unwrap_or(f64::NAN), &x.to_string())
            }
            Prim::F64 => {
                let x = f64::from_le_bytes(self.num::<8>()?);
                float(x, &x.to_string())
            }
            Prim::Str => {
                let len = u32::from_le_bytes(self.num::<4>()?) as usize;
                let bytes = self.take(len)?;
                let bytes = bytes.strip_suffix(&[0]).unwrap_or(bytes);
                let text = String::from_utf8_lossy(bytes);
                match text.char_indices().nth(MAX_STRING_CHARS) {
                    Some((cut, _)) => Value::String(format!("{}…", &text[..cut])),
                    None => Value::String(text.into_owned()),
                }
            }
        })
    }
}

/// A JSON number, or the text `NaN`/`inf` that JSON cannot hold.
fn float(x: f64, text: &str) -> Value {
    Number::from_f64(x).map_or_else(|| Value::String(text.to_string()), Value::Number)
}

#[cfg(test)]
mod tests {
    use super::super::parse::parse;
    use super::*;
    use serde_json::json;
    use std::collections::HashMap;

    fn defs(list: &[(&str, &str)]) -> HashMap<String, Arc<MsgDef>> {
        list.iter()
            .map(|(name, text)| {
                let pkg = name.split('/').next().unwrap();
                (name.to_string(), Arc::new(parse(pkg, text).unwrap()))
            })
            .collect()
    }

    fn run(defs: &HashMap<String, Arc<MsgDef>>, root: &str, data: &[u8]) -> Result<Value, String> {
        decode(data, root, &|n| defs.get(n).cloned())
    }

    const LE: [u8; 4] = [0, 1, 0, 0];

    #[test]
    fn decodes_clock() {
        let d = defs(&[
            ("rosgraph_msgs/Clock", "builtin_interfaces/Time clock\n"),
            (
                "builtin_interfaces/Time",
                "int32 sec\nuint32 nanosec\n",
            ),
        ]);
        let mut data = LE.to_vec();
        data.extend(1i32.to_le_bytes());
        data.extend(500u32.to_le_bytes());
        assert_eq!(
            run(&d, "rosgraph_msgs/Clock", &data).unwrap(),
            json!({"clock": {"sec": 1, "nanosec": 500}})
        );
    }

    #[test]
    fn aligns_strings_and_sequences() {
        let d = defs(&[(
            "p/M",
            "uint8 a\nuint32 b\nstring c\nint16[] d\nfloat64 e\nbool[2] f\nfloat32 g\n",
        )]);
        let mut data = LE.to_vec();
        data.push(7); // a, offset 0
        data.extend([0; 3]); // pad to 4
        data.extend(9u32.to_le_bytes()); // b, 4
        data.extend(3u32.to_le_bytes()); // c length with NUL, 8
        data.extend(*b"hi\0"); // 12..15
        data.push(0); // pad to 16
        data.extend(2u32.to_le_bytes()); // d count, 16
        data.extend((-1i16).to_le_bytes()); // 20
        data.extend(2i16.to_le_bytes()); // 22
        data.extend(1.5f64.to_le_bytes()); // e, 24 (already aligned)
        data.extend([1, 0]); // f, 32
        data.extend([0; 2]); // pad to 36
        data.extend(0.1f32.to_le_bytes()); // g
        assert_eq!(
            run(&d, "p/M", &data).unwrap(),
            json!({"a": 7, "b": 9, "c": "hi", "d": [-1, 2], "e": 1.5, "f": [true, false], "g": 0.1})
        );
    }

    #[test]
    fn reads_big_endian() {
        let d = defs(&[("p/M", "uint32 x\n")]);
        let data = [0, 0, 0, 0, 0, 0, 1, 2];
        assert_eq!(run(&d, "p/M", &data).unwrap(), json!({"x": 258}));
    }

    #[test]
    fn long_primitive_arrays_are_summarised() {
        let d = defs(&[("p/M", "uint8[] data\nint32 after\n")]);
        let mut data = LE.to_vec();
        data.extend(100_000u32.to_le_bytes());
        data.extend(vec![0u8; 100_000]);
        data.extend(5i32.to_le_bytes());
        assert_eq!(
            run(&d, "p/M", &data).unwrap(),
            json!({"data": "<100000 x uint8>", "after": 5})
        );
    }

    #[test]
    fn empty_message_has_a_dummy_byte() {
        let d = defs(&[("std_msgs/Empty", "")]);
        let mut data = LE.to_vec();
        data.push(0);
        assert_eq!(run(&d, "std_msgs/Empty", &data).unwrap(), json!({}));
    }

    #[test]
    fn refuses_what_does_not_fit() {
        let d = defs(&[("p/M", "int32 a\nint32[] b\n")]);
        // Too short.
        assert!(run(&d, "p/M", &[0, 1, 0, 0, 1, 0]).is_err());
        // Sequence count larger than the payload.
        let mut data = LE.to_vec();
        data.extend(1i32.to_le_bytes());
        data.extend(u32::MAX.to_le_bytes());
        assert!(run(&d, "p/M", &data).is_err());
        // Bytes left over: the definition is probably the wrong one.
        let mut data = LE.to_vec();
        data.extend([0; 8]);
        data.extend(0u32.to_le_bytes());
        assert!(run(&d, "p/M", &data).is_err());
        // Not CDR, and a definition that is not loaded.
        assert!(run(&d, "p/M", &[0, 6, 0, 0, 0, 0, 0, 0]).is_err());
        assert!(run(&d, "p/Other", &[0, 1, 0, 0, 0]).is_err());
    }

    #[test]
    fn non_finite_floats_become_text() {
        let d = defs(&[("p/M", "float64 x\n")]);
        let mut data = LE.to_vec();
        data.extend(f64::NAN.to_le_bytes());
        assert_eq!(run(&d, "p/M", &data).unwrap(), json!({"x": "NaN"}));
    }
}
