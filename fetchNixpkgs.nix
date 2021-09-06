{ rev    ? "fc0e373cf9740acb99f2c4d38e97bbba48aa9f9c"             # The Git revision of nixpkgs to fetch
, sha256 ? "09wraic21dg37jr3zrq4wyj9x62rrbvgmi7zyaa3mnr7kmjw86ci" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
