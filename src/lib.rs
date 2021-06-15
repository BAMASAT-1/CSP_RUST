
use std::vec::Vec;
use std::boxed::Box;
use socketcan::CANSocket;
use serial::prelude::*;

pub mod CSP {
    // https://github.com/libcsp/libcsp/blob/master/include/csp/csp_types.h
    //----------------------------------------------------------------
    //  Reserved Ports for CSP Service
    enum csp_service_ports_t {
        CSP_CMP				= 0,   // !< CSP management, e.g. memory, routes, stats
        CSP_PING			= 1,   // !< Ping - return ping
        CSP_PS				= 2,   // !< Current process list
        CSP_MEMFREE			= 3,   // !< Free memory
        CSP_REBOOT			= 4,   // !< Reboot, see #CSP_REBOOT_MAGIC and #CSP_REBOOT_SHUTDOWN_MAGIC
        CSP_BUF_FREE		= 5,   // !< Free CSP buffers
        CSP_UPTIME			= 6,   // !< Uptime
    }
    //----------------------------------------------------------------
    // listen on all prots
    pub const CSP_ANY:u8 = 255;
    //------------------------------
    // message priority
    enum csp_prio_t {
        CSP_PRIO_CRITICAL		= 0, // !< Critical
        CSP_PRIO_HIGH			= 1, // !< High
        CSP_PRIO_NORM			= 2, // !< Normal (default);
        CSP_PRIO_LOW			= 3, // !< Low
    }

    //--------------------------------------------------------------
    //FLAGS
    pub const CSP_ID_PRIO_SIZE:u8  =		0b0000_0010; // !< Bits for priority, see #csp_prio_t
    pub const CSP_ID_HOST_SIZE:u8  =		0b0000_0101; // !< Bits for host (destination/source address);
    pub const CSP_ID_PORT_SIZE:u8  =		0b0000_0110; // !< Bits for port (destination/source port);
    pub const CSP_ID_FLAGS_SIZE:u8 =		0b0000_1000; // !< Bits for flags, see @ref CSP_HEADER_FLAGS
    // ---------------------------------------------------------------------------
    // header masks
    pub const CSP_ID_PRIO_MAX     :u32=	((1 << (CSP_ID_PRIO_SIZE)) - 1);  // !< Max priority value in header
    pub const CSP_ID_HOST_MAX	  :u32=	((1 << (CSP_ID_HOST_SIZE)) - 1);  // !< Max host value in header
    pub const CSP_ID_PORT_MAX	  :u32=	((1 << (CSP_ID_PORT_SIZE)) - 1);  // !< Max port value in header
    pub const CSP_ID_FLAGS_MAX	  :u32=	((1 << (CSP_ID_FLAGS_SIZE)) - 1); // !< Max flag(s) value in header

    /** CSP identifier/header - priority mask */
    pub const CSP_ID_PRIO_MASK	  :u32=	(CSP_ID_PRIO_MAX  << (CSP_ID_FLAGS_SIZE + (2 * CSP_ID_PORT_SIZE) + (2 * CSP_ID_HOST_SIZE)));
    /** CSP identifier/header - source address mask */
    pub const CSP_ID_SRC_MASK	  :u32=	(CSP_ID_HOST_MAX  << (CSP_ID_FLAGS_SIZE + (2 * CSP_ID_PORT_SIZE) + (1 * CSP_ID_HOST_SIZE)));
    /** CSP identifier/header - destination address mask */
    pub const CSP_ID_DST_MASK	  :u32=	(CSP_ID_HOST_MAX  << (CSP_ID_FLAGS_SIZE + (2 * CSP_ID_PORT_SIZE)));
    /** CSP identifier/header - destination port mask */
    pub const CSP_ID_DPORT_MASK   :u32=	(CSP_ID_PORT_MAX  << (CSP_ID_FLAGS_SIZE + (1 * CSP_ID_PORT_SIZE)));
    /** CSP identifier/header - source port mask */
    pub const CSP_ID_SPORT_MASK   :u32=	(CSP_ID_PORT_MAX  << (CSP_ID_FLAGS_SIZE));
    /** CSP identifier/header - flag mask */
    pub const CSP_ID_FLAGS_MASK	  :u32=	(CSP_ID_FLAGS_MAX << (0));
    /** CSP identifier/header - connection mask (source & destination address + source & destination ports) */
    pub const CSP_ID_CONN_MASK	  :u32=	(CSP_ID_SRC_MASK | CSP_ID_DST_MASK | CSP_ID_DPORT_MASK | CSP_ID_SPORT_MASK);
    // ----------------------------------------------------------------
    // CSP HEADER FLAGS
    pub const CSP_FRES1:u32 = 0x80;
    pub const CSP_FRES2:u32 = 0x40;
    pub const CSP_FRES3:u32 = 0x20;
    pub const CSP_FFRAG:u32 = 0x10;
    pub const CSP_FHMAC:u32 = 0x08;
    pub const CSP_FXTEA:u32 = 0x04;
    pub const CSP_FRDP :u32 = 0x02;
    pub const CSP_FCRC32:u32 = 0x01;
    //----------------------------------------------------------------
    // CSP Socket Options
    pub const CSP_SO_NONE       :u32=			0x0000; // !< No socket options
    pub const CSP_SO_RDPREQ     :u32=			0x0001; // !< Require RDP
    pub const CSP_SO_RDPPROHIB	:u32=        	0x0002; // !< Prohibit RDP
    pub const CSP_SO_HMACREQ	:u32=            0x0004; // !< Require HMAC
    pub const CSP_SO_HMACPROHIB	:u32=    	    0x0008; // !< Prohibit HMAC
    pub const CSP_SO_XTEAREQ	:u32=		    0x0010; // !< Require XTEA
    pub const CSP_SO_XTEAPROHIB	:u32=	        0x0020; // !< Prohibit HMAC
    pub const CSP_SO_CRC32REQ	:u32=		    0x0040; // !< Require CRC32
    pub const CSP_SO_CRC32PROHIB:u32=		    0x0080; // !< Prohibit CRC32
    pub const CSP_SO_CONN_LESS	:u32=	        0x0100; // !< Enable Connection Less mode
    pub const CSP_SO_INTERNAL_LISTEN :u32=       0x1000; // !< Internal flag: listen called on socket
    // ------------------------------
    // CSP Connection Options
    pub const CSP_O_NONE    :u32=  		CSP_SO_NONE;      // !< No connection options
    pub const CSP_O_RDP		:u32= 	    CSP_SO_RDPREQ;      // !< Enable RDP
    pub const CSP_O_NORDP	:u32= 		CSP_SO_RDPPROHIB;   // !< Disable RDP
    pub const CSP_O_HMAC	:u32= 		CSP_SO_HMACREQ;     // !< Enable HMAC
    pub const CSP_O_NOHMAC	:u32= 		CSP_SO_HMACPROHIB;  // !< Disable HMAC
    pub const CSP_O_XTEA	:u32= 		CSP_SO_XTEAREQ;     // !< Enable XTEA
    pub const CSP_O_NOXTEA	:u32= 		CSP_SO_XTEAPROHIB;  // !< Disable XTEA
    pub const CSP_O_CRC32	:u32= 		CSP_SO_CRC32REQ;    // !< Enable CRC32
    pub const CSP_O_NOCRC32	:u32= 		CSP_SO_CRC32PROHIB; // !< Disable CRC32
    // ------------------------------
    // CSP Padding Bytes
    pub const CSP_PADDING_BYTES: usize = 10;

    
    pub union csp_id_t {
        
        ext: u8,

        // possible bugs from endian of type u8 (assuming big endian)
        
        //struct __attribute__((__packed__)) line 127 in csp_types.h
        pri:u8,
        src:u8,  
        dst:u8,
        dport:u8,
        sport:u8,
        flags:u8
    }

    pub struct csp_packet_t {
        padding: [u8; CSP_PADDING_BYTES],
        length: usize,
        id: csp_id_t,

        // using box pointers to heap to have dynamically sized data packets
        data8: Box<[u8]>,   // for can
        data16: Box<[u16]>, // for uart
        data32: Box<[u32]>, // for anything else (networking)

    }

    pub const CSP_REBOOT_MAGIC:u32 = 0x80078007;
    pub const CSP_REBOOT_SHUTDOWN_MAGIC:u32 = 0xD1E5529A;

    // create csp_packet_t structs that have data split into [u8], [u16], [u32]
    // depending on the destination ie (can, uart, internal network)
    pub fn csp8u_CAN<T> (packet: csp_packet_t, data:T) -> Option<csp_packet_t> {

    }

    pub fn csp16u_UART<T> (data:T) -> Option<csp_packet_t>{

    }

    impl csp_packet_t {

        pub fn send_CAN(cansocket: socketcan::CANSocket){

        }

        pub fn send_UART(packet: csp_packet_t){

        }

        pub fn recv_CAN() -> Result<csp_packet_t>{
            
        }

        pub fn recv_uart() -> Result<csp_packet_t>{

        }

        pub fn to_json(packet: csp_packet_t) {

        }

    }
}
