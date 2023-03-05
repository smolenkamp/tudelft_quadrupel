/// Turns the non-blocking expression `$e` into a blocking expression, with a maximum number of
/// retries `max`.
///
/// Just like [`nb::block`](https://docs.rs/nb/latest/nb/macro.block.html), `$e` is evaluated in a
/// loop, except now if the loop does more than `max` iterations, `None` is returned, and `Some(_)`
/// is returned instead of `_`.
#[macro_export]
macro_rules! block_retry {
    ($e:expr, $max:expr) => {
        {
        let mut i = $max;
        loop {
            #[allow(unreachable_patterns)]
            match $e {
                Err($crate::nb::Error::Other(e)) =>
                {
                    #[allow(unreachable_code)]
                    break Err(e)
                }
                //wb @ Err($crate::nb::Error::WouldBlock) => {
                Err($crate::nb::Error::WouldBlock) => {
                    if i == 0 {
                        break Ok(None)
                    } else {
                        i-=1;
                    }
                }
                Ok(x) => break Ok(Some(x)),
            }
        }
        }
    };
}
