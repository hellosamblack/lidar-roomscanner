# Current State
- Fixed firmware freeze when Ethernet cable is replugged (lwIP `mdns_resp_add_netif` double-add assertion).
- Fixed host `UdpSource` stream recovery by adding an active mDNS re-query when stream data stops (instead of broadcast).

# Next steps
- Monitor stability of Ethernet stream.
