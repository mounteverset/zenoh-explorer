//! Parser for ROS 2 `.msg` definition files.

/// Primitive field types. `byte` and `char` are `U8`, as in rosidl.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Prim {
    Bool,
    U8,
    I8,
    U16,
    I16,
    U32,
    I32,
    U64,
    I64,
    F32,
    F64,
    Str,
}

impl Prim {
    /// Size in bytes on the wire; for `Str` the minimum (its length prefix).
    pub fn size(self) -> usize {
        match self {
            Prim::Bool | Prim::U8 | Prim::I8 => 1,
            Prim::U16 | Prim::I16 => 2,
            Prim::U32 | Prim::I32 | Prim::F32 | Prim::Str => 4,
            Prim::U64 | Prim::I64 | Prim::F64 => 8,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Prim::Bool => "bool",
            Prim::U8 => "uint8",
            Prim::I8 => "int8",
            Prim::U16 => "uint16",
            Prim::I16 => "int16",
            Prim::U32 => "uint32",
            Prim::I32 => "int32",
            Prim::U64 => "uint64",
            Prim::I64 => "int64",
            Prim::F32 => "float32",
            Prim::F64 => "float64",
            Prim::Str => "string",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Base {
    Prim(Prim),
    /// Another message, as `package/Name`.
    Nested(String),
}

/// Array form of a field. Bounded sequences look like unbounded ones on the wire.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Arr {
    One,
    Fixed(usize),
    Seq,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Field {
    pub name: String,
    pub base: Base,
    pub arr: Arr,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MsgDef {
    pub fields: Vec<Field>,
}

impl MsgDef {
    /// Names of the messages this one contains.
    pub fn nested(&self) -> impl Iterator<Item = &str> {
        self.fields.iter().filter_map(|f| match &f.base {
            Base::Nested(n) => Some(n.as_str()),
            Base::Prim(_) => None,
        })
    }
}

/// True for a non-empty `[A-Za-z0-9_]` name. Names from the network end up in
/// file paths and URLs, so everything that is not this is refused.
pub fn is_ident(s: &str) -> bool {
    !s.is_empty() && s.chars().all(|c| c.is_ascii_alphanumeric() || c == '_')
}

/// Parse the text of `<pkg>/msg/<Name>.msg`. Constants and default values are skipped.
pub fn parse(pkg: &str, text: &str) -> Result<MsgDef, String> {
    let mut fields = Vec::new();
    for (n, raw) in text.lines().enumerate() {
        let line = strip_comment(raw).trim();
        if line.is_empty() {
            continue;
        }
        let err = |m: &str| format!("line {}: {}", n + 1, m);
        let (ty, rest) = line
            .split_once(char::is_whitespace)
            .ok_or_else(|| err("expected `type name`"))?;
        let rest = rest.trim_start();
        let name_end = rest
            .find(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
            .unwrap_or(rest.len());
        let name = &rest[..name_end];
        if rest[name_end..].trim_start().starts_with('=') {
            continue; // constant
        }
        if name.is_empty() {
            return Err(err("missing field name"));
        }
        let (base, arr) = parse_type(pkg, ty).map_err(|m| err(&m))?;
        fields.push(Field {
            name: name.to_string(),
            base,
            arr,
        });
    }
    Ok(MsgDef { fields })
}

/// Cut a trailing `# comment`, ignoring a `#` inside quotes.
fn strip_comment(line: &str) -> &str {
    let mut quote: Option<char> = None;
    for (i, c) in line.char_indices() {
        match (quote, c) {
            (None, '#') => return &line[..i],
            (None, '"' | '\'') => quote = Some(c),
            (Some(q), _) if q == c => quote = None,
            _ => {}
        }
    }
    line
}

fn parse_type(pkg: &str, ty: &str) -> Result<(Base, Arr), String> {
    let (ty, arr) = match ty.strip_suffix(']') {
        Some(head) => {
            let (ty, inner) = head
                .split_once('[')
                .ok_or_else(|| format!("bad array type `{ty}`"))?;
            let arr = if inner.is_empty() || inner.starts_with("<=") {
                Arr::Seq
            } else {
                Arr::Fixed(inner.parse().map_err(|_| format!("bad array size `{inner}`"))?)
            };
            (ty, arr)
        }
        None => (ty, Arr::One),
    };
    // `string<=N`: the bound does not change the encoding.
    let ty = match ty.split_once("<=") {
        Some((head, _)) if head == "string" => head,
        _ => ty,
    };
    let prim = match ty {
        "bool" => Some(Prim::Bool),
        "byte" | "char" | "uint8" => Some(Prim::U8),
        "int8" => Some(Prim::I8),
        "uint16" => Some(Prim::U16),
        "int16" => Some(Prim::I16),
        "uint32" => Some(Prim::U32),
        "int32" => Some(Prim::I32),
        "uint64" => Some(Prim::U64),
        "int64" => Some(Prim::I64),
        "float32" => Some(Prim::F32),
        "float64" => Some(Prim::F64),
        "string" => Some(Prim::Str),
        _ => None,
    };
    if let Some(p) = prim {
        return Ok((Base::Prim(p), arr));
    }
    if ty.starts_with("wstring") {
        return Err("wstring is not supported".into());
    }
    // `pkg/Name`, `pkg/msg/Name`, or `Name` in the same package.
    let name = match ty.split('/').collect::<Vec<_>>()[..] {
        [name] => format!("{pkg}/{name}"),
        [p, name] | [p, "msg", name] => format!("{p}/{name}"),
        _ => return Err(format!("bad type `{ty}`")),
    };
    if !name.split('/').all(is_ident) {
        return Err(format!("bad type `{ty}`"));
    }
    Ok((Base::Nested(name), arr))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn f(name: &str, base: Base, arr: Arr) -> Field {
        Field {
            name: name.into(),
            base,
            arr,
        }
    }

    #[test]
    fn parses_twist() {
        let def = parse(
            "geometry_msgs",
            "# velocity\n\nVector3  linear\nVector3  angular # trailing\n",
        )
        .unwrap();
        let v = Base::Nested("geometry_msgs/Vector3".into());
        assert_eq!(
            def.fields,
            vec![
                f("linear", v.clone(), Arr::One),
                f("angular", v, Arr::One)
            ]
        );
    }

    #[test]
    fn skips_constants_and_defaults_keeps_arrays() {
        let def = parse(
            "p",
            "uint8 FOO=1\nstring BAR = \"a # b\"\nint32 x 5\nfloat64[36] cov\nint8[] a\nint8[<=4] b\nstring<=10 s\nstd_msgs/Header header\nfoo/msg/Bar bar\nbyte flag\n",
        )
        .unwrap();
        let names: Vec<_> = def.fields.iter().map(|f| f.name.as_str()).collect();
        assert_eq!(names, ["x", "cov", "a", "b", "s", "header", "bar", "flag"]);
        assert_eq!(def.fields[1].arr, Arr::Fixed(36));
        assert_eq!(def.fields[2].arr, Arr::Seq);
        assert_eq!(def.fields[3].arr, Arr::Seq);
        assert_eq!(def.fields[4].base, Base::Prim(Prim::Str));
        assert_eq!(def.fields[5].base, Base::Nested("std_msgs/Header".into()));
        assert_eq!(def.fields[6].base, Base::Nested("foo/Bar".into()));
        assert_eq!(def.fields[7].base, Base::Prim(Prim::U8));
    }

    #[test]
    fn refuses_bad_types() {
        assert!(parse("p", "wstring s").is_err());
        assert!(parse("p", "../x/Evil e").is_err());
        assert!(parse("p", "int32").is_err());
    }
}
