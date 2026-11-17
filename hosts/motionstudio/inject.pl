#!/usr/bin/env perl
# Inject a Rhai script into MotionStudio's profile TOML, for every FAAL filter
# copy (default layer + per-layer copies share the same custom filter id).
#
# Usage:
#   perl inject.pl <script.rhai> [--dry] [--only 0,1] [--profile PATH]
#
# Defaults to the profile named in settings.json (selected_profile). Writes the
# profile, its .bak twin, and a timestamped rollback copy; then re-parses the
# result with tomllib and restores the original if anything looks wrong.
#
# Why a file and not a one-liner: the script has to be re-encoded as a TOML
# basic string (\r escapes plus real newlines), and doing that inside bash -
# PowerShell nesting costs more escapes than it saves.

use strict;
use warnings;
use File::Copy qw(copy);
use POSIX qw(strftime);

my $ROOT = "$ENV{APPDATA}/io.hotuns.motionstudio";

my $script = shift @ARGV or die usage();
my %opt = (only => undef, dry => 0, profile => undef);
while (@ARGV) {
    my $a = shift @ARGV;
    if    ($a eq '--dry')     { $opt{dry} = 1 }
    elsif ($a eq '--profile') { $opt{profile} = shift @ARGV }
    elsif ($a eq '--only')    { $opt{only} = { map { $_ => 1 } split /,/, shift @ARGV } }
    else                      { die "unknown argument: $a\n" . usage() }
}
die "no such script: $script\n" unless -f $script;

my $profile = $opt{profile} || active_profile();
die "profile not found: $profile\n" unless -f $profile;

# --- encode the script the way the app does ---------------------------------
my $src = slurp($script);
$src =~ s/\r\n/\n/g;
$src =~ s/\r/\n/g;
die "script contains a triple quote, which would close the TOML string\n"
    if $src =~ /"""/;
(my $enc = $src) =~ s/\\/\\\\/g;
$enc =~ s/\n/\\r\n/g;

# --- pick the FAAL filter blocks -------------------------------------------
my $text = slurp($profile);
my @part = split /(?=^\[\[)/m, $text;
my @faal;
for my $i (0 .. $#part) {
    next unless $part[$i] =~ /^\[\[[^\n]*custom_filters\]\]/;
    next unless $part[$i] =~ /^\s*name\s*=\s*"faal"/m;
    push @faal, $i;
}
die "no FAAL filter found in $profile\n" unless @faal;

my @targets = $opt{only}
    ? grep { $opt{only}{$_} } 0 .. $#faal
    : 0 .. $#faal;
die "no filter matched --only (have " . scalar(@faal) . " FAAL filters)\n" unless @targets;

for my $n (@targets) {
    my $i = $faal[$n];
    my $before = $part[$i];
    $part[$i] =~ s/(source\s*=\s*""")(.*?)(""")/$1 . $enc . $3/se
        or die "could not find source field in FAAL filter #$n\n";
    die "filter #$n has no source field change?\n" if $part[$i] eq $before;
}

my $out = join '', @part;
my $replaced = join ',', @targets;

print "profile   : $profile\n";
print "script    : $script (" . ($src =~ tr/\n//) . " lines)\n";
print "filters   : " . scalar(@faal) . " FAAL copies, replacing #$replaced\n";

if ($opt{dry}) {
    print "dry run, nothing written\n";
    exit 0;
}

# --- write .toml, .bak twin, timestamped rollback --------------------------
my $stamp = strftime('%Y%m%d-%H%M%S', localtime);
my $rollback = "$profile.before-inject-$stamp";
copy($profile, $rollback) or die "rollback copy failed: $!\n";
spew($profile, $out);
copy($profile, "$profile.bak") if -f "$profile.bak";

# --- validate by re-parsing --------------------------------------------------
my $py = $ENV{PYTHON} || 'python';
my $check = q{
import sys, tomllib
def walk(n, out):
    if isinstance(n, dict):
        if n.get('name') == 'faal' and 'source' in n:
            out.append(n)
        for v in n.values():
            walk(v, out)
    elif isinstance(n, list):
        for v in n:
            walk(v, out)
d = tomllib.load(open(sys.argv[1], 'rb'))
out = []
walk(d, out)
want = open(sys.argv[2], encoding='utf-8').read().splitlines()
picked = [int(x) for x in sys.argv[3].split(',')]
bad = [i for i in picked if out[i]['source'].splitlines() != want]
print('tomllib: parsed, %d FAAL copies, %d matched, mismatch %s'
      % (len(out), len(picked) - len(bad), bad))
sys.exit(1 if bad or len(out) == 0 else 0)
};
my $ok = system($py, '-c', $check, $profile, $script, $replaced) == 0;
if (!$ok) {
    copy($rollback, $profile) or die "restore failed: $!\n";
    copy($profile, "$profile.bak") if -f "$profile.bak";
    die "validation failed, profile restored from $rollback\n";
}
print "rollback  : $rollback\n";
print "done\n";

sub slurp {
    my $p = shift;
    open my $fh, '<:raw', $p or die "open $p: $!\n";
    local $/;
    my $s = <$fh>;
    close $fh;
    return $s;
}

sub spew {
    my ($p, $s) = @_;
    open my $fh, '>:raw', $p or die "write $p: $!\n";
    print $fh $s;
    close $fh or die "close $p: $!\n";
}

sub active_profile {
    my $settings = slurp("$ROOT/settings.json");
    my ($id) = $settings =~ /"selected_profile"\s*:\s*"([^"]+)"/;
    die "no selected_profile in settings.json\n" unless $id;
    return "$ROOT/profiles/$id.toml";
}

sub usage {
    return "usage: perl inject.pl <script.rhai> [--dry] [--only 0,1] [--profile PATH]\n";
}
